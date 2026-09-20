//! The full-screen front-end: `klippy-client console` on a terminal.
//!
//! Three panes and a keyboard:
//!
//! ```text
//! ● ready · Printer is ready
//! 1 (list_endpoints)
//!   {
//!     "endpoints": ["list_endpoints"]
//!   }
//! < {"id": null, "method": "klippy:status", "params": {...}}
//! klippy> objects/query {"objects": {"toolhead": ["position"]}}
//! Enter send · ↑↓ history · PgUp/PgDn/Home/End log · ^↑/^↓ line · ^G g-code · .help · ^C quit
//! ```
//!
//! The header tracks the printer's state, the log holds everything that
//! happened, and the last line is the hint. Nothing is bordered: the window is
//! usually a few dozen rows of printer output, and every column counts.
//!
//! Everything the session does arrives as an [`Entry`] and is appended to the
//! log pane; the window exists to make that log readable while the printer is
//! running. Pushes are the reason: a subscription keeps printing under a line
//! mode session too, but only a window can show it without fighting the prompt
//! for the same line.
//!
//! ## Scrolling
//!
//! The log supports scrolling to review past output:
//!
//! - **Mouse wheel**: scroll up/down by 3 lines
//! - **Click/drag the scrollbar**: go to that part of the log
//! - **^S** (or `.mouse`, or a click on the log's text): hand the mouse back to
//!   the terminal so text can be selected and copied there; any key takes it
//!   back. The view freezes while it is released, since a terminal's selection
//!   is anchored to the screen and a line arriving would slide it off.
//! - **PgUp/PgDn**: scroll by a page
//! - **Home**: jump to the top (oldest lines)
//! - **End**: jump to the bottom (newest line)
//! - **↑/↓**: walk the typed-line history at the bottom; once the log is
//!   scrolled back they move it a line at a time, which `Ctrl+↑/↓` always does
//! - When scrolled back, new entries do not move the view — your place is kept
//!   until you return to the bottom.
//!
//! The unit is a rendered line, not a logged entry: a single entry wrapped over
//! several rows can be read a line at a time, and one keystroke moves what the
//! eye counts, not what the protocol happened to delimit. A log line's level is
//! coloured and the line itself is not, the way `tracing` prints it.
//!
//! The log's rightmost column is a scrollbar. It is always reserved, so the text
//! never shifts sideways when the log outgrows the pane, and the thumb appears
//! only when there is something to scroll. Because the log pane itself owns that
//! column, it is one column narrower than the window.
//!
//! # Threads and tasks
//!
//! One task (the one running [`run`]) draws and owns all the state, one blocking
//! thread reads keystrokes, and the session's socket reads are raced against
//! those keystrokes in the same `select!`. The keyboard thread polls with a
//! timeout rather than blocking forever, so that leaving the window does not
//! leave a thread asleep in `read` — which would also hold up the runtime's
//! shutdown.
//!
//! # Not covered
//!
//! * Selection and copying are the terminal's, not the window's — but a
//!   terminal only selects with a mouse it owns, and the window needs the mouse
//!   for its wheel and its scrollbar. `^S` / `.mouse` / a click on the log gives
//!   it back, and the next key takes it again.
//! * No reconnection: the window closes when the server goes away, after
//!   printing why. Reconnecting would mean re-establishing every subscription.
//! * The log keeps everything for the life of the session; a very chatty
//!   subscription will grow it without bound.

use std::cell::{Cell, RefCell};
use std::io::IsTerminal as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ratatui::crossterm::event::{
    self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers, MouseButton, MouseEvent,
    MouseEventKind,
};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Paragraph, Scrollbar, ScrollbarOrientation, ScrollbarState};
use ratatui::Frame;
use serde_json::Value;

use klippy_api::address::ApiTarget;
use klippy_api::TransportError;

use crate::session::{self, Control, Entry, LogLevel, Notice, Output, Session};

/// How often the keyboard thread wakes to check whether it should stop.
///
/// It cannot be interrupted out of `event::read`, so the thread polls instead;
/// this is the price of leaving the window instantly.
const KEY_POLL: Duration = Duration::from_millis(100);

/// How long to keep reading after the window is closing, so replies already on
/// their way are not lost.
const LEAVE_GRACE: Duration = Duration::from_secs(1);

/// How many entries the log keeps before dropping the oldest.
const LOG_LIMIT: usize = 2_000;

/// Open a window on `target`.
///
/// # Errors
/// Returns [`TransportError`] if the server cannot be reached or goes away.
pub async fn run(target: ApiTarget) -> Result<(), TransportError> {
    let session = Session::connect(target).await?;
    run_session(session, None).await
}

/// Show a window on an existing session.
///
/// `host_log` is where a host that embedded this window sends its own log
/// records (see the host's `logging` module). They are interleaved with the
/// protocol traffic in the log pane, which is the point: in one window, what the
/// client asked, what the server answered, and what the host said about itself
/// belong in the order they happened.
///
/// # Errors
/// Returns [`TransportError`] if the connection goes away. Dropping the future —
/// the host shutting down under it, say — restores the terminal just the same,
/// because the guard that does so is dropped with it.
pub async fn run_session(
    session: Session,
    host_log: Option<tokio::sync::mpsc::UnboundedReceiver<Entry>>,
) -> Result<(), TransportError> {
    // Taken before anything can fail: a panic inside the window, or the future
    // being dropped, must still give the terminal back.
    let _terminal = TerminalGuard::take();
    let terminal = ratatui::init();
    let outcome = event_loop(session, terminal, host_log).await;
    outcome
}

/// Gives the terminal back when the window ends, however it ends.
///
/// `ratatui::init` installs a panic hook that restores the terminal, but nothing
/// restores it when the future is simply dropped — which is what happens when
/// the host shuts down while the window is up. A guard does, because dropping
/// the future drops its locals.
///
/// Mouse reporting is not part of what `ratatui::restore` puts back — it is a
/// mode of its own — so it is given back here first. A window that ended while
/// holding the mouse would otherwise leave the terminal sending mouse escapes to
/// whatever ran next.
struct TerminalGuard;

impl TerminalGuard {
    /// Take the terminal, restoring it when the returned guard is dropped.
    fn take() -> Self {
        Self
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        set_mouse_capture(false);
        ratatui::restore();
    }
}

/// Whether this process can draw a window at all.
///
/// Both ends have to be a terminal: a window drawn into a pipe would be a
/// screenful of escape codes.
pub fn is_available() -> bool {
    std::io::stdin().is_terminal() && std::io::stdout().is_terminal()
}

/// The window's state: everything the next frame is drawn from.
struct App {
    entries: Vec<Entry>,
    input: Input,
    /// How many rendered lines the viewport is scrolled back from the bottom
    /// (0 = the newest line). Counted in lines, not entries, so a wrapped entry
    /// scrolls a row at a time.
    scroll: usize,
    /// Height of the log pane as last drawn, so PgUp/PgDn move a real page.
    viewport: Cell<usize>,
    /// Width of the log pane as last drawn. A pinned view needs it to count the
    /// lines a new entry adds below the viewport.
    width: Cell<usize>,
    /// The whole log's rendered height, so the scrollbar can show how much of
    /// it is on screen without re-wrapping the log every frame.
    heights: RefCell<Heights>,
    /// The scrollbar's column as last drawn, so a click can find it. Zero-sized
    /// when the log fits and no bar is drawn.
    gutter: Cell<Rect>,
    /// The log pane as last drawn, scrollbar column included, so a click on the
    /// text can be told from one on the bar.
    log: Cell<Rect>,
    /// Whether a scrollbar drag is in progress. The button holds the drag, not
    /// the pointer's column, so this survives the pointer leaving the bar.
    dragging: bool,
    /// Whether the window has taken the mouse. While it has, the wheel scrolls
    /// and the scrollbar drags — and the terminal cannot select text, because it
    /// never sees the drag. `^S` / `.mouse` hands it back.
    mouse: bool,
    /// The connection's state, shown in the header.
    status: Status,
    /// Set by a local command that asked to leave.
    quit: bool,
    /// Whether typed lines are sent as G-Code (`gcode/script`).
    gcode: bool,
    /// Whether this window has already subscribed to G-Code output; every
    /// `gcode/subscribe_output` registers another output handler.
    gcode_subscribed: bool,
    /// Whether the handshake's `info` has been answered.
    ///
    /// Until it has, an `info` reply is the header's business rather than the
    /// log's — but only that one: a later, typed `info` is the user's, and it
    /// belongs in the log like any other reply.
    greeted: bool,
    /// How message bodies are shown: YAML by default, JSON on demand.
    format: Format,
}

/// What the header says about the connection.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Status {
    /// Connected, with whatever `info` reported.
    Connected { state: String, message: String },
    /// Connected, but `info` did not answer usefully.
    Unknown,
    /// The connection is gone.
    Closed(String),
}

/// How the window shows a message body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Format {
    /// Block YAML: a tree with no quoting, the default.
    #[default]
    Yaml,
    /// Compact JSON: the wire form, for when the exact bytes matter.
    Json,
}

impl App {
    fn new() -> Self {
        Self {
            entries: Vec::new(),
            input: Input::default(),
            scroll: 0,
            viewport: Cell::new(0),
            width: Cell::new(0),
            heights: RefCell::new(Heights::default()),
            gutter: Cell::new(Rect::new(0, 0, 0, 0)),
            log: Cell::new(Rect::new(0, 0, 0, 0)),
            dragging: false,
            mouse: true,
            status: Status::Unknown,
            quit: false,
            gcode: false,
            gcode_subscribed: false,
            greeted: false,
            format: Format::Yaml,
        }
    }

    /// Hand the mouse to the terminal, so it can select text.
    ///
    /// The window holds the mouse — the wheel and the scrollbar need it — so
    /// letting go is the move that has to be asked for, and this is how: `^S`,
    /// `.mouse`, or a click on the log's text. Holding it again is not asked
    /// for at all: a key does that (see [`App::mouse_for_key`]), which is just
    /// as well — a terminal with the mouse no longer sends the window a click
    /// to go on.
    ///
    /// Nothing is written to the log, deliberately: releasing freezes the view
    /// (`push`), so a line about it would land under the frozen pane where the
    /// reader cannot see it. The footer says it instead.
    fn release_mouse(&mut self) {
        self.mouse = false;
    }

    /// Take the mouse back for the window, so the wheel and the scrollbar work.
    fn capture_mouse(&mut self) {
        self.mouse = true;
    }

    /// Settle the mouse for a keypress, before the key itself is interpreted.
    ///
    /// Any key takes the mouse back: a reader at the keyboard is not selecting,
    /// so the window's gestures are what they want. `^S` is the one key that
    /// then hands it straight out again, being the keyboard's own way of asking
    /// for the terminal's selection.
    fn mouse_for_key(&mut self, code: KeyCode, ctrl: bool) {
        self.capture_mouse();
        if ctrl && code == KeyCode::Char('s') {
            self.release_mouse();
        }
    }

    /// Flip between request mode and g-code mode.
    fn toggle_gcode(&mut self) {
        self.gcode = !self.gcode;
        let text = if self.gcode {
            "g-code mode: typed lines go to gcode/script (^G or .gcode to leave)"
        } else {
            "request mode: typed lines are requests"
        };
        self.push(Entry::notice(Notice::Info, text));
    }

    /// Subscribe to G-Code output the first time g-code mode is entered.
    ///
    /// Subscribing is what makes `respond_info` and errors visible: without it
    /// the only sign of a command is its reply.
    async fn ensure_gcode_subscription(
        &mut self,
        session: &mut Session,
    ) -> Result<(), TransportError> {
        if !self.gcode || self.gcode_subscribed {
            return Ok(());
        }
        session.subscribe_gcode_output(self).await?;
        self.gcode_subscribed = true;
        Ok(())
    }

    fn push(&mut self, entry: Entry) {
        // A viewport scrolled back is anchored to the lines it is showing, not
        // to the bottom: the new entry lands below it, so the offset grows by
        // the lines that entry takes. At the bottom (`scroll == 0`) the view
        // follows the new entry, which is what a log should do.
        //
        // A released mouse is the same case with a different reason. The
        // reader is selecting text with the terminal, whose selection is
        // anchored to the screen rather than to the text: a line arriving would
        // slide the selection off what it was drawn around. So the view freezes
        // — the offset grows, and the same lines stay where they are.
        let width = self.width.get().max(1);
        let height = entry_height(&entry, width, self.format);
        let added = if self.scroll > 0 || !self.mouse {
            height
        } else {
            0
        };

        // The heights parallel the entries. A pane resize or a body-format
        // switch re-wraps the whole log; anything that shortened the entries
        // without touching the heights (`Ctrl+L`) shows up here too. Measuring
        // one entry per push is what lets the scrollbar show the whole log
        // without re-wrapping it.
        let mut measured = self.heights.borrow_mut();
        if measured.width != width
            || measured.format != self.format
            || measured.heights.len() != self.entries.len()
        {
            measured.remeasure(&self.entries, width, self.format);
        }

        self.entries.push(entry);
        measured.push(height);

        if self.entries.len() > LOG_LIMIT {
            let dropped = self.entries.len() - LOG_LIMIT;
            self.entries.drain(..dropped);
            measured.drop_front(dropped);
        }
        drop(measured);

        self.scroll = self.scroll.saturating_add(added);
    }

    /// Move the log for a scrolling key, and say whether the key was one.
    ///
    /// `Ctrl+↑/↓` scrolls a line from anywhere. The plain arrows scroll only
    /// once the log is scrolled back: at the bottom they walk the typed-line
    /// history, which is what the hand expects there.
    fn scroll_key(&mut self, code: KeyCode, ctrl: bool) -> bool {
        match (code, ctrl) {
            (KeyCode::PageUp, _) => {
                self.scroll = self.scroll.saturating_add(self.viewport.get().max(1));
            }
            (KeyCode::PageDown, _) => {
                self.scroll = self.scroll.saturating_sub(self.viewport.get().max(1));
            }
            (KeyCode::Home, _) => {
                // Top = oldest. The renderer clamps this to the top of the log,
                // so it fills the pane with the oldest lines rather than
                // leaving it empty.
                self.scroll = usize::MAX;
            }
            (KeyCode::End, _) => self.scroll = 0,
            (KeyCode::Up, _) if ctrl || self.scroll > 0 => {
                self.scroll = self.scroll.saturating_add(1);
            }
            (KeyCode::Down, _) if ctrl || self.scroll > 0 => {
                self.scroll = self.scroll.saturating_sub(1);
            }
            _ => return false,
        }
        true
    }

    /// The scrollbar track row a mouse cell points at, if it points at one.
    ///
    /// The bar is one column at the right edge of the log pane; the row is
    /// counted from the top of that pane. Nothing to scroll means nothing to
    /// hit.
    fn gutter_row(&self, column: u16, row: u16) -> Option<usize> {
        let gutter = self.gutter.get();
        if gutter.width == 0 || gutter.height == 0 {
            return None;
        }
        if column != gutter.x || row < gutter.y || row >= gutter.y + gutter.height {
            return None;
        }
        Some((row - gutter.y) as usize)
    }

    /// Whether a mouse cell is in the log's text rather than its scrollbar.
    ///
    /// The bar owns the pane's last column; everything to its left is text.
    fn in_log_body(&self, column: u16, row: u16) -> bool {
        let log = self.log.get();
        if log.width <= 1 || row < log.y || row >= log.y + log.height {
            return false;
        }
        column >= log.x && column < log.x + log.width - 1
    }

    /// The track row a pointer is on while dragging, clamped to the track.
    ///
    /// Once a drag has begun the pointer is allowed to wander off the bar —
    /// sideways out of its column, or above and below the pane. The button is
    /// what holds the drag, so a row outside the track means the top or the
    /// bottom of the log rather than losing the drag.
    fn clamp_track_row(&self, row: u16) -> usize {
        let gutter = self.gutter.get();
        if gutter.height == 0 {
            return 0;
        }
        row.saturating_sub(gutter.y).min(gutter.height - 1) as usize
    }

    /// Scroll so the pointer's place on the track is in view.
    ///
    /// `track_row` is counted from the top of the log pane. The thumb's middle
    /// follows the pointer, so grabbing the thumb does not make the log jump
    /// out from under the cursor.
    fn scroll_to_track(&mut self, track_row: usize) {
        let height = self.gutter.get().height as usize;
        if height == 0 {
            return;
        }
        let total = self.total_lines(self.width.get().max(1));
        let shown = height.min(total);
        let range = total.saturating_sub(shown);
        if range == 0 {
            self.scroll = 0;
            return;
        }

        // The thumb's length, near enough to what the widget draws for the hit
        // area to feel right. `total > shown`, so this cannot divide by zero.
        let thumb = (shown * height / total).clamp(1, height);
        let travel = height.saturating_sub(thumb);
        // A track with no room to move has only one place to be.
        let first = match travel {
            0 => 0,
            travel => {
                track_row
                    .saturating_sub(thumb / 2)
                    .min(travel)
                    .saturating_mul(range)
                    / travel
            }
        };
        // `first` counts from the top; `scroll` counts from the bottom.
        self.scroll = range.saturating_sub(first);
    }

    /// The whole log's rendered height at `width`, the number a scrollbar wants.
    ///
    /// Wrapping is the only reason this differs from `entries.len()`, and it is
    /// not free, so the per-entry heights are kept and only measured again when
    /// the pane width, the body format, or the set of entries changed.
    fn total_lines(&self, width: usize) -> usize {
        let width = width.max(1);
        let mut measured = self.heights.borrow_mut();
        // The heights parallel the entries; if anything emptied one without the
        // other (`Ctrl+L`), the two disagree and the log is measured again.
        if measured.width != width
            || measured.format != self.format
            || measured.heights.len() != self.entries.len()
        {
            measured.remeasure(&self.entries, width, self.format);
        }
        measured.total
    }

    /// Handle a local command only the window has.
    ///
    /// Returns whether the line was one; the session's own commands
    /// (`.subscribe`, `.quit`) go to the session. `.yaml` and `.json` are the
    /// window's because the line front-end has nothing to switch — it is one
    /// compact JSON line per event by design — and `.help` is answered here so
    /// that the window's commands sit in the same list as the session's.
    fn window_command(&mut self, line: &str) -> bool {
        match line.trim() {
            ".gcode" => {
                self.toggle_gcode();
                true
            }
            ".mouse" => {
                self.release_mouse();
                true
            }
            ".yaml" | ".json" => {
                self.format = if line.trim() == ".json" {
                    Format::Json
                } else {
                    Format::Yaml
                };
                let name = match self.format {
                    Format::Yaml => "YAML",
                    Format::Json => "JSON",
                };
                self.push(Entry::notice(
                    Notice::Info,
                    format!("message bodies are now {name}"),
                ));
                true
            }
            ".help" => {
                self.push(Entry::notice(
                    Notice::Info,
                    format!(
                        "{}\n\nWindow:\n  .yaml / .json   show message bodies as YAML or JSON\n  .gcode          toggle g-code mode (^G): typed lines go to gcode/script\n  .mouse          hand the mouse back to the terminal (^S, or click the log) so text can be selected",
                        session::usage()
                    ),
                ));
                true
            }
            _ => false,
        }
    }
}

impl Output for App {
    fn write(&mut self, entry: Entry) {
        match &entry {
            // The handshake's `info` is the header's business, not the log's:
            // showing it twice would be noise.
            Entry::Reply(reply) if !self.greeted => {
                self.greeted = true;
                if reply.method.as_deref() == Some("info") && !reply.is_error() {
                    let result = reply.result().cloned().unwrap_or(Value::Null);
                    self.status = Status::Connected {
                        state: field(&result, "state"),
                        message: field(&result, "state_message").replace('\n', " "),
                    };
                    return;
                }
            }
            // After that it is the `webhooks` object that reports the state, on
            // every change — so a subscription keeps the header honest without
            // anyone asking.
            Entry::Push(message) => {
                let status = message
                    .get("params")
                    .and_then(|params| params.get("status"))
                    .and_then(|status| status.get("webhooks"));
                if let Some(status) = status {
                    let state = status.get("state").and_then(Value::as_str);
                    let message = status.get("state_message").and_then(Value::as_str);
                    if let (Some(state), Some(message)) = (state, message) {
                        self.status = Status::Connected {
                            state: state.to_string(),
                            message: message.replace('\n', " "),
                        };
                    }
                }
            }
            _ => (),
        }
        self.push(entry);
    }
}

/// A string field of a JSON object, or `?`.
fn field(value: &Value, name: &str) -> String {
    value
        .get(name)
        .and_then(Value::as_str)
        .unwrap_or("?")
        .to_string()
}

/// Take the terminal's mouse for the window, or give it back.
///
/// A window that has the mouse gets the wheel and the scrollbar; a terminal that
/// has it gets its own selection. Only one of them can, which is why `^S` exists.
fn set_mouse_capture(captured: bool) {
    use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture};

    let _ = if captured {
        ratatui::crossterm::execute!(std::io::stdout(), EnableMouseCapture)
    } else {
        ratatui::crossterm::execute!(std::io::stdout(), DisableMouseCapture)
    };
}

/// Draw, read keys, read the socket, until one of them says stop.
async fn event_loop(
    mut session: Session,
    mut terminal: ratatui::DefaultTerminal,
    mut host_log: Option<tokio::sync::mpsc::UnboundedReceiver<Entry>>,
) -> Result<(), TransportError> {
    let mut app = App::new();

    // A host that embedded this window logged things before it existed — config,
    // listeners, and so on. Those lines happened first, so they go first; they
    // are the beginning of the story the window is telling.
    if let Some(logs) = host_log.as_mut() {
        while let Ok(entry) = logs.try_recv() {
            app.write(entry);
        }
    }

    app.push(Entry::notice(
        Notice::Info,
        format!("Connected to {}.", session.label()),
    ));
    session.handshake(&mut app).await?;

    let (mut keys, keyboard) = spawn_keyboard();
    // The window holds the mouse unless the reader has handed it back, and the
    // terminal has to be told which way it is. The last state applied is kept
    // here so the escape sequence goes out on a change, not every frame.
    let mut captured = false;
    let outcome = loop {
        if app.mouse != captured {
            captured = app.mouse;
            set_mouse_capture(captured);
        }
        terminal
            .draw(|frame| draw(frame, &app))
            .expect("drawing failed");

        // Nothing here borrows `app`: the futures are built before the branches
        // run, and the branches then own both `app` and `session` freely.
        enum Step {
            Key(Option<Event>),
            Message(Result<crate::connection::Incoming, TransportError>),
            HostLog(Option<Entry>),
        }
        let step = tokio::select! {
            key = keys.recv() => Step::Key(key),
            message = session.receive() => Step::Message(message),
            // A host that embedded this window keeps talking about itself while
            // the printer runs; its lines go into the same log, in order.
            entry = async { host_log.as_mut()?.recv().await }, if host_log.is_some() => {
                Step::HostLog(entry)
            }
        };

        match step {
            // The keyboard thread is gone, which only happens on shutdown.
            Step::Key(None) => break Ok(()),
            Step::Key(Some(Event::Key(key))) => {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                match handle_key(&mut app, &mut session, key).await? {
                    Control::Continue => (),
                    Control::Quit => break Ok(()),
                }
            }
            Step::Key(Some(Event::Mouse(mouse))) => {
                handle_mouse(&mut app, mouse);
            }
            Step::Key(Some(_)) => (),
            // A closed channel means the host stopped logging; the window keeps
            // working, it just has nothing more to say about itself.
            Step::HostLog(None) => host_log = None,
            Step::HostLog(Some(entry)) => app.write(entry),
            Step::Message(Ok(message)) => app.write(message.into()),
            Step::Message(Err(err)) => {
                app.status = Status::Closed(err.to_string());
                app.push(Entry::notice(Notice::Failure, err.to_string()));
                break Err(err);
            }
        }

        if app.quit {
            break Ok(());
        }
    };

    // Leaving is not the same as abandoning: replies to what was just typed are
    // still owed, and the window is still up to show them for a moment.
    session.drain(&mut app, LEAVE_GRACE).await;
    terminal
        .draw(|frame| draw(frame, &app))
        .expect("drawing failed");

    stop_keyboard(keyboard);
    outcome
}

/// One keypress.
async fn handle_key(
    app: &mut App,
    session: &mut Session,
    key: KeyEvent,
) -> Result<Control, TransportError> {
    let ctrl = key.modifiers.contains(KeyModifiers::CONTROL);
    // The mouse settles first, and this key still does its own job after.
    app.mouse_for_key(key.code, ctrl);
    match (key.code, ctrl) {
        // Leaving: ^C, ^D and Esc all mean "I am done", and all of them are
        // what a terminal user will try.
        (KeyCode::Char('c') | KeyCode::Char('d'), true) | (KeyCode::Esc, _) => {
            return Ok(Control::Quit)
        }
        (KeyCode::Char('l'), true) => {
            app.entries.clear();
            app.scroll = 0;
        }
        // Switching what a typed line means: a request, or G-Code.
        (KeyCode::Char('g'), true) => {
            app.toggle_gcode();
            app.ensure_gcode_subscription(session).await?;
        }
        (KeyCode::Enter, _) => {
            let line = app.input.take();
            if line.is_empty() {
                return Ok(Control::Continue);
            }
            app.scroll = 0;
            if app.window_command(&line) {
                // Entering g-code mode is the window's business, but the output
                // subscription it needs belongs to the session.
                app.ensure_gcode_subscription(session).await?;
            } else {
                let outcome = if app.gcode {
                    session.handle_gcode_line(&line, app).await?
                } else {
                    session.handle_line(&line, app).await?
                };
                match outcome {
                    Control::Continue => (),
                    Control::Quit => app.quit = true,
                }
            }
        }
        // Scrolling the log, which is the reason the panes exist: the printer's
        // own output would otherwise push everything else away. Whatever this
        // is not, the input line owns.
        (code, _) => {
            if !app.scroll_key(code, ctrl) {
                app.input.edit(code, ctrl);
            }
        }
    }
    Ok(Control::Continue)
}

/// Handle mouse events (wheel scrolling, and the scrollbar as a target).
fn handle_mouse(app: &mut App, mouse: MouseEvent) {
    match mouse.kind {
        MouseEventKind::ScrollUp => {
            // Scroll up = view older content.
            app.scroll = app.scroll.saturating_add(3);
        }
        MouseEventKind::ScrollDown => {
            // Scroll down = view newer content.
            app.scroll = app.scroll.saturating_sub(3);
        }
        // A press on the bar takes hold of it. A press anywhere else lets go:
        // only the bar starts a drag.
        MouseEventKind::Down(MouseButton::Left) => match app.gutter_row(mouse.column, mouse.row) {
            Some(track) => {
                app.dragging = true;
                app.scroll_to_track(track);
            }
            // A click on the text is a request for the text, not for the wheel:
            // the window lets the terminal have the mouse so it can select, and
            // `^S` takes it back. (The gesture that follows is a second one — a
            // terminal only selects with a drag it saw from the beginning.)
            None if app.in_log_body(mouse.column, mouse.row) => {
                app.dragging = false;
                app.release_mouse();
            }
            None => app.dragging = false,
        },
        // The drag follows the pointer's row even when it leaves the bar's
        // column; `gutter_row` would have dropped it there.
        MouseEventKind::Drag(MouseButton::Left) if app.dragging => {
            app.scroll_to_track(app.clamp_track_row(mouse.row));
        }
        MouseEventKind::Up(MouseButton::Left) => app.dragging = false,
        _ => (),
    }
}

/// The panes.
///
/// The header's height is not fixed: a state message can be a sentence (a
/// shutdown reason usually is), and the status bar is the only place it is
/// shown, so it wraps and the log starts below it. It never takes so much that
/// the log, the input and the footer lose their rows.
fn draw(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let mut header = header_lines(app, area.width as usize);
    header.truncate(area.height.saturating_sub(3).max(1) as usize);
    let header_height = header.len() as u16;

    let [header_area, log, input, footer] = Layout::vertical([
        Constraint::Length(header_height),
        Constraint::Min(1),
        Constraint::Length(1),
        Constraint::Length(1),
    ])
    .areas(area);

    frame.render_widget(Paragraph::new(Text::from(header)), header_area);
    draw_log(frame, app, log);
    draw_input(frame, app, input);
    draw_footer(frame, app, footer);
}

/// What the header says: the marker, the text, and the style of both.
fn header(app: &App) -> (&'static str, String, Style) {
    match &app.status {
        Status::Connected { state, message } => (
            match state.as_str() {
                "ready" => "●",
                "startup" => "◌",
                _ => "▲",
            },
            format!("{state} · {message}"),
            match state.as_str() {
                "ready" => Style::new().fg(Color::Green),
                "startup" => Style::new().fg(Color::Yellow),
                _ => Style::new().fg(Color::Red),
            },
        ),
        Status::Unknown => (
            "◌",
            "state unknown".to_string(),
            Style::new().add_modifier(Modifier::DIM),
        ),
        Status::Closed(why) => (
            "✕",
            format!("disconnected: {why}"),
            Style::new().fg(Color::Red),
        ),
    }
}

/// The header's lines, wrapped to `width` columns.
///
/// The marker leads only the first line; the rest are indented by its width so
/// the state reads as one block.
fn header_lines(app: &App, width: usize) -> Vec<Line<'static>> {
    let (marker, text, style) = header(app);
    let prefix = format!("{marker} ");
    let indent = prefix.chars().count();
    wrap(&text, width.saturating_sub(indent).max(1))
        .into_iter()
        .enumerate()
        .map(|(index, chunk)| {
            let lead = if index == 0 {
                Span::styled(prefix.clone(), style)
            } else {
                Span::raw(" ".repeat(indent))
            };
            Line::from(vec![
                lead,
                Span::styled(chunk, style.add_modifier(Modifier::BOLD)),
            ])
        })
        .collect()
}

fn draw_log(frame: &mut Frame, app: &App, area: Rect) {
    // The rightmost column belongs to the scrollbar, so the log wraps one
    // column short of the pane. Reserving it even when the log fits keeps the
    // text from jumping sideways the moment the log outgrows the pane.
    let [text, gutter] =
        Layout::horizontal([Constraint::Min(1), Constraint::Length(1)]).areas(area);

    // Remember the pane's shape: PgUp/PgDn move a page, a pinned viewport
    // counts new lines at this width, and the heights are measured at it. The
    // whole pane is remembered too, so a click can be placed — the bar owns its
    // last column and the rest is text.
    app.log.set(area);
    app.viewport.set(text.height as usize);
    app.width.set(text.width as usize);

    let width = text.width as usize;
    let total = app.total_lines(width);
    let view = visible_lines(app, width, text.height as usize, total);
    frame.render_widget(Paragraph::new(Text::from(view.lines)), text);

    if total > text.height as usize {
        // The bar is a target as well as a picture: remembering where it was
        // drawn is what lets a click land on it.
        app.gutter.set(gutter);
        let mut state = ScrollbarState::new(total)
            .position(scrollbar_position(view.first, total, view.shown))
            .viewport_content_length(view.shown);
        frame.render_stateful_widget(
            Scrollbar::new(ScrollbarOrientation::VerticalRight)
                // Arrow heads would eat a two-row track whole, and the window
                // has no use for them.
                .begin_symbol(None)
                .end_symbol(None)
                .track_symbol(Some("│"))
                .thumb_symbol("█")
                .track_style(Style::new().fg(Color::DarkGray))
                .thumb_style(Style::new().fg(Color::Gray)),
            gutter,
            &mut state,
        );
    } else {
        // Nothing to scroll: the column stays, but it is not a target.
        app.gutter.set(Rect::new(0, 0, 0, 0));
    }
}

fn draw_input(frame: &mut Frame, app: &App, area: Rect) {
    let prompt = if session::is_local(&app.input.text()) {
        // A local command is the client's own, so it is marked as such before it
        // is even sent.
        Span::styled("local> ", Style::new().fg(Color::Magenta))
    } else if app.gcode {
        // In g-code mode the whole line is the script.
        Span::styled("gcode> ", Style::new().fg(Color::Yellow))
    } else {
        Span::styled("klippy> ", Style::new().fg(Color::Blue))
    };
    let prompt_width = prompt.content.chars().count() as u16;

    // Only the tail of a long line fits; the cursor decides which tail.
    let available = area.width.saturating_sub(prompt_width).max(1) as usize;
    let (shown, cursor_column) = app.input.window(available);

    frame.render_widget(
        Paragraph::new(Line::from(vec![prompt, Span::raw(shown)])),
        area,
    );
    frame.set_cursor_position((area.x + prompt_width + cursor_column as u16, area.y));
}

fn draw_footer(frame: &mut Frame, app: &App, area: Rect) {
    // How far back the log really is. `scroll` can be past the top (`Home` is
    // `usize::MAX`) and a log that fits cannot be scrolled at all, so the offset
    // is clamped before it is shown — and so that one line of scrolling is
    // still one visible word in the hint.
    let range = app
        .total_lines(app.width.get().max(1))
        .saturating_sub(app.viewport.get().max(1));
    let back = app.scroll.min(range);

    let hint = if app.quit {
        "leaving…".to_string()
    } else if !app.mouse {
        // A released mouse is a mode the reader has to remember: the gestures
        // they just used no longer do anything, and the terminal's do.
        "mouse released · drag to select · any key takes it back · ^C quit".to_string()
    } else if back > 0 {
        let lines = if back == 1 { "line" } else { "lines" };
        format!(
            "viewing older entries · {back} {lines} back · ↑↓ scroll · End bottom · Home top · ^C quit"
        )
    } else if app.gcode {
        "g-code mode · Enter send · ^G request mode · .gcode · ^C quit".to_string()
    } else {
        "Enter send · ↑↓ history · PgUp/PgDn/Home/End log · ^↑/^↓ line · ^G g-code · .help · ^C quit"
            .to_string()
    };
    frame.render_widget(
        Paragraph::new(Line::from(hint)).style(Style::new().add_modifier(Modifier::DIM)),
        area,
    );
}

/// The log's rendered height, entry by entry.
///
/// The scrollbar needs the whole log's height, and wrapping every entry to get
/// it would be the most expensive thing the window does — the log is capped at
/// [`LOG_LIMIT`] entries, so it is not a small number. The heights are therefore
/// kept: one entry per [`App::push`], the whole log only when the pane width or
/// body format changes, since either one re-wraps everything.
#[derive(Default)]
struct Heights {
    /// The pane width these heights were measured at (0 = never measured).
    width: usize,
    /// The body format they were measured in.
    format: Format,
    /// One height per entry, parallel to `App::entries`.
    heights: Vec<usize>,
    /// The sum of `heights`.
    total: usize,
}

impl Heights {
    fn remeasure(&mut self, entries: &[Entry], width: usize, format: Format) {
        self.width = width;
        self.format = format;
        self.heights.clear();
        self.total = 0;
        for entry in entries {
            let height = entry_height(entry, width, format);
            self.heights.push(height);
            self.total += height;
        }
    }

    fn push(&mut self, height: usize) {
        self.heights.push(height);
        self.total += height;
    }

    fn drop_front(&mut self, count: usize) {
        self.total -= self.heights.drain(..count).sum::<usize>();
    }
}

/// How many rendered lines an entry takes at `width`.
///
/// The renderer lays text out exactly this way, so this is both the scrollbar's
/// arithmetic and what keeps a pinned viewport over the same lines when new
/// output arrives.
fn entry_height(entry: &Entry, width: usize, format: Format) -> usize {
    wrap(&entry_text(entry, format), width.max(1)).len()
}

/// Where a viewport starting at `first` belongs on the scrollbar's track.
///
/// The widget spreads `position` over `0..total-1`, but a `shown`-line viewport
/// can only start as late as `total - shown`, so handing it `first` directly
/// would stop the thumb short of the bottom when the log is following. Stretch
/// the offset over the range the bar actually has, and both ends stay flush.
fn scrollbar_position(first: usize, total: usize, shown: usize) -> usize {
    match total.checked_sub(shown) {
        Some(range) if range > 0 => first.saturating_mul(total - 1) / range,
        _ => 0,
    }
}

/// One frame's worth of the log: the lines to draw, and where they sit in the
/// whole log — which is what a scrollbar reports.
struct Viewport {
    /// The lines to draw, oldest first.
    lines: Vec<Line<'static>>,
    /// The first shown line's index, counted from the top of the log.
    first: usize,
    /// How many lines the pane shows.
    shown: usize,
}

/// The lines the log pane shows, oldest first.
///
/// `scroll` is how many rendered lines the pane is scrolled back from the
/// bottom (0 = the newest line). The pane shows `height` lines ending there,
/// clamped to the top of the log, so scrolling past the end shows the oldest
/// lines rather than an empty pane. `total` is the whole log's height, which
/// `visible_lines` itself has no reason to measure.
fn visible_lines(app: &App, width: usize, height: usize, total: usize) -> Viewport {
    let width = width.max(1);
    let want = app.scroll;

    // Collect the newest lines, newest first, until the pane and the scrolled-
    // past lines fit. `usize::MAX` (Home) collects the whole log, which the
    // clamp below turns into the top of it.
    let cap = height.saturating_add(want);
    let total_messages = app.entries.iter().filter(|entry| is_message(entry)).count();
    let mut messages_seen = 0;
    let mut lines: Vec<Line<'static>> = Vec::new();
    'entries: for entry in app.entries.iter().rev() {
        // The colour alternates by absolute message index, counted from the
        // newest, so a message keeps its colour however far the log is
        // scrolled.
        let index = if is_message(entry) {
            let index = total_messages - messages_seen - 1;
            messages_seen += 1;
            index
        } else {
            0
        };
        let style = entry_style(entry, index);
        let wrapped = wrap(&entry_text(entry, app.format), width);
        for (position, text) in wrapped.into_iter().enumerate().rev() {
            lines.push(entry_line(entry, text, position == 0, style));
            if lines.len() >= cap {
                break 'entries;
            }
        }
    }

    // `lines` is newest-first. The pane ends `skip` lines above the bottom and
    // reaches `height` lines further up. Clamping `skip` is what keeps a
    // scrolled-to-the-end view showing the oldest lines instead of nothing.
    let skip = want.min(lines.len().saturating_sub(height));
    lines.drain(..skip);
    lines.truncate(height);
    let shown = lines.len();
    lines.reverse();

    Viewport {
        lines,
        // `skip` lines were dropped from the bottom, and `shown` reach up from
        // there, so this is the topmost line's place in the whole log.
        first: total.saturating_sub(shown + skip),
        shown,
    }
}

/// One wrapped line of an entry, styled.
///
/// A log line is the exception: only its level is coloured, and the line itself
/// is left plain — what the host's own `tracing` output does, and what keeps a
/// pane of prose readable. Everything else (a message body, a notice, a failed
/// reply) is styled whole, because its colour is the only thing marking it.
///
/// `first` says whether this is the entry's first line, since that is the one
/// `entry_text` puts the tag in front of.
fn entry_line(entry: &Entry, text: String, first: bool, style: Style) -> Line<'static> {
    let Entry::Log { level, .. } = entry else {
        return Line::from(Span::styled(text, style));
    };
    if !first {
        return Line::from(Span::raw(text));
    }

    // The tag `entry_text` wrote, and the space after it. A pane too narrow for
    // the tag colours what fits rather than dropping the tag's colour.
    let tag_width = level.tag().chars().count() + 1;
    if text.chars().count() <= tag_width {
        return Line::from(Span::styled(text, style));
    }
    let mut chars = text.chars();
    let tag: String = chars.by_ref().take(tag_width).collect();
    let rest: String = chars.collect();
    Line::from(vec![Span::styled(tag, style), Span::raw(rest)])
}

/// How an entry looks.
///
/// Messages alternate between two colours — `message_index` is the message's
/// position among messages, not among entries — so two messages in a row never
/// look alike however many log lines sit between them. A failed request is red
/// instead: it is the one thing that must not blend in. Log lines and notices
/// keep their own colours.
fn entry_style(entry: &Entry, message_index: usize) -> Style {
    match entry {
        // A failed request is the thing the user has to notice.
        Entry::Reply(reply) if reply.is_error() => Style::new().fg(Color::Red),
        Entry::Sent { .. } | Entry::Reply(_) | Entry::Push(_) => {
            Style::new().fg(MESSAGE_COLORS[message_index % MESSAGE_COLORS.len()])
        }
        Entry::Log { level, .. } => match level {
            // The five levels keep the colours the host's own `tracing` output
            // uses on a terminal — purple, blue, green, yellow, red — so a line
            // reads the same in both places. Every line also carries its tag, so
            // the colour is a second cue rather than the only one.
            LogLevel::Trace => Style::new().fg(Color::Magenta),
            LogLevel::Debug => Style::new().fg(Color::Blue),
            LogLevel::Info => Style::new().fg(Color::Green),
            LogLevel::Warn => Style::new().fg(Color::Yellow),
            LogLevel::Error => Style::new().fg(Color::Red),
        },
        Entry::Notice { kind, .. } => match kind {
            // The window's everyday voice is plain: the colours belong to the
            // host's lines and to the printer's messages, and that keeps the
            // pane free of a sixth hue that means nothing in particular.
            Notice::Info => Style::new(),
            // A hint or a problem keeps the hue of the level it means, the way
            // `Warn` and `Error` share it with them.
            Notice::Hint => Style::new().fg(Color::Yellow),
            Notice::Problem => Style::new().fg(Color::Red),
            Notice::Failure => Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
        },
    }
}

/// The two colours a message alternates between.
///
/// Two different hues, not two shades of white: ANSI 7 and 15 are the same
/// colour in many themes, which is exactly the “I cannot see any colour” a pair
/// of greys produces. Plain ANSI foregrounds rather than a 256-colour
/// background, so they work wherever colour is shown at all.
///
/// Neither is a hue the five log levels use (`TRACE`…`ERROR`), so a message is
/// never mistaken for a log line at a glance.
const MESSAGE_COLORS: [Color; 2] = [Color::Cyan, Color::White];

/// Whether an entry is a message, rather than the window talking to itself.
fn is_message(entry: &Entry) -> bool {
    matches!(entry, Entry::Sent { .. } | Entry::Reply(_) | Entry::Push(_))
}

/// What the window shows for an entry.
///
/// A message body is a tree, and the window renders it as [`Format::Yaml`] by
/// default — the shape a tree is read in, without the quoting a JSON line needs
/// — behind a `<` (received) or `>` (sent) marker. Everything already written as
/// a sentence (an error, a notice, a log line) is left to [`Entry::text`]; the
/// line front-end uses that for all of it, deliberately: it wants one compact
/// JSON line per event.
fn entry_text(entry: &Entry, format: Format) -> String {
    match entry {
        Entry::Sent { message, .. } => marked('>', &body(message, format)),
        Entry::Reply(reply) if !reply.is_error() => marked('<', &body(&reply.message, format)),
        Entry::Push(message) => marked('<', &body(message, format)),
        other => other.text(),
    }
}

/// Put a `<`/`>` marker in front of a body, indenting the rest under it.
fn marked(marker: char, body: &str) -> String {
    let mut lines = body.lines();
    let first = lines.next().unwrap_or_default();
    let mut text = format!("{marker} {first}");
    for line in lines {
        text.push('\n');
        text.push_str("  ");
        text.push_str(line);
    }
    text
}

/// One value in the window's current [`Format`].
fn body(value: &Value, format: Format) -> String {
    match format {
        Format::Yaml => yaml(value),
        // The wire form, which is also the last resort of `yaml`.
        Format::Json => serde_json::to_string(value).unwrap_or_else(|_| "null".to_string()),
    }
}

/// One value as block-style YAML, without the trailing newline.
///
/// YAML is what the window shows, but it is never what goes on the wire, so a
/// value it cannot write still has to be shown: the compact JSON is the
/// fallback, and `null` the fallback's fallback.
fn yaml(value: &Value) -> String {
    match serde_yaml::to_string(value) {
        Ok(text) => text.trim_end().to_string(),
        Err(_) => serde_json::to_string(value).unwrap_or_else(|_| "null".to_string()),
    }
}

/// Break `text` into lines of at most `width` columns.
///
/// Counts characters rather than displaying them, which is right for JSON and
/// wrong only for wide glyphs — of which a JSON reply has none.
fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    for raw in text.split('\n') {
        if raw.is_empty() {
            lines.push(String::new());
            continue;
        }
        let mut current = String::new();
        let mut used = 0;
        for character in raw.chars() {
            if used == width {
                lines.push(std::mem::take(&mut current));
                used = 0;
            }
            current.push(character);
            used += 1;
        }
        lines.push(current);
    }
    lines
}

// ===========================================================================
// Input
// ===========================================================================

/// The editable line at the bottom of the window.
///
/// The terminal is in raw mode, so every editing key a user expects has to be
/// implemented here: there is no line discipline left to do it.
#[derive(Default)]
pub struct Input {
    buffer: Vec<char>,
    /// Where the next character goes, in characters from the start.
    cursor: usize,
    history: Vec<String>,
    /// Where `Up`/`Down` currently are in `history`, counting back from the end.
    recall: Option<usize>,
}

impl Input {
    /// The line as it stands.
    pub fn text(&self) -> String {
        self.buffer.iter().collect()
    }

    /// Whether there is nothing to send.
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Take the line, remembering it for `Up`.
    pub fn take(&mut self) -> String {
        let line = self.text();
        if !line.trim().is_empty() {
            self.history.push(line.clone());
        }
        self.buffer.clear();
        self.cursor = 0;
        self.recall = None;
        line
    }

    /// Apply one key to the line.
    pub fn edit(&mut self, code: KeyCode, ctrl: bool) {
        match (code, ctrl) {
            (KeyCode::Char(character), false) => self.insert(character),
            (KeyCode::Char('a'), true) => self.cursor = 0,
            (KeyCode::Char('e'), true) => self.cursor = self.buffer.len(),
            // The usual way to abandon a half-typed line.
            (KeyCode::Char('u'), true) => {
                self.buffer.clear();
                self.cursor = 0;
            }
            (KeyCode::Backspace, _) => {
                if self.cursor > 0 {
                    self.cursor -= 1;
                    self.buffer.remove(self.cursor);
                }
            }
            (KeyCode::Delete, _) => {
                if self.cursor < self.buffer.len() {
                    self.buffer.remove(self.cursor);
                }
            }
            (KeyCode::Left, _) => self.cursor = self.cursor.saturating_sub(1),
            (KeyCode::Right, _) => {
                self.cursor = (self.cursor + 1).min(self.buffer.len());
            }
            (KeyCode::Home, _) => self.cursor = 0,
            (KeyCode::End, _) => self.cursor = self.buffer.len(),
            (KeyCode::Up, _) => self.recall_older(),
            (KeyCode::Down, _) => self.recall_newer(),
            _ => (),
        }
    }

    fn insert(&mut self, character: char) {
        self.buffer.insert(self.cursor, character);
        self.cursor += 1;
    }

    /// Step back through what was typed before.
    fn recall_older(&mut self) {
        if self.history.is_empty() {
            return;
        }
        let index = match self.recall {
            None => self.history.len() - 1,
            Some(0) => 0,
            Some(index) => index - 1,
        };
        self.recall = Some(index);
        self.buffer = self.history[index].chars().collect();
        self.cursor = self.buffer.len();
    }

    /// Step forward again, ending at the empty line that was being typed.
    fn recall_newer(&mut self) {
        let Some(index) = self.recall else { return };
        if index + 1 >= self.history.len() {
            self.recall = None;
            self.buffer.clear();
        } else {
            self.recall = Some(index + 1);
            self.buffer = self.history[index + 1].chars().collect();
        }
        self.cursor = self.buffer.len();
    }

    /// The part of the line that fits in `width` columns, and where the cursor
    /// lands inside it.
    fn window(&self, width: usize) -> (String, usize) {
        if self.buffer.len() <= width {
            return (self.text(), self.cursor);
        }
        // Scroll just enough to keep the cursor visible.
        let start = self.cursor.saturating_sub(width.saturating_sub(1));
        let shown: String = self.buffer[start..].iter().take(width).collect();
        (shown, self.cursor - start)
    }
}

// ===========================================================================
// Keyboard
// ===========================================================================

/// A thread reading keys, and the flag that stops it.
struct Keyboard {
    stop: Arc<AtomicBool>,
    thread: std::thread::JoinHandle<()>,
}

/// Read terminal events on a thread of their own.
///
/// `event::read` blocks, and nothing can interrupt it, so the thread polls with
/// a timeout: that way the flag is noticed within [`KEY_POLL`], and the runtime's
/// shutdown is not held up by a thread asleep in a read.
fn spawn_keyboard() -> (tokio::sync::mpsc::UnboundedReceiver<Event>, Keyboard) {
    let (sender, receiver) = tokio::sync::mpsc::unbounded_channel();
    let stop = Arc::new(AtomicBool::new(false));
    let stopping = Arc::clone(&stop);

    let handle = std::thread::Builder::new()
        .name("klippy-client-keys".to_string())
        .spawn(move || {
            while !stopping.load(Ordering::Relaxed) {
                match event::poll(KEY_POLL) {
                    Ok(true) => match event::read() {
                        Ok(event) => {
                            if sender.send(event).is_err() {
                                break;
                            }
                        }
                        Err(_) => break,
                    },
                    Ok(false) => continue,
                    Err(_) => break,
                }
            }
        })
        .expect("cannot spawn the keyboard thread");

    (
        receiver,
        Keyboard {
            stop,
            thread: handle,
        },
    )
}

/// Ask the keyboard thread to stop, and wait for it to notice.
///
/// Joining is not politeness: the thread holds a `read` on the terminal, and
/// leaving it in flight while the terminal is restored is how a client ends up
/// eating the shell's next few keystrokes.
fn stop_keyboard(keyboard: Keyboard) {
    keyboard.stop.store(true, Ordering::Relaxed);
    // The thread polls with a timeout, so this waits at most `KEY_POLL` — not
    // for a keypress.
    let _ = keyboard.thread.join();
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    /// Draw the window into a buffer and return it as text, one string per row.
    fn render(app: &App, width: u16, height: u16) -> Vec<String> {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(|frame| draw(frame, app)).unwrap();
        let buffer = terminal.backend().buffer().clone();
        (0..height)
            .map(|row| {
                (0..width)
                    .map(|column| buffer[(column, row)].symbol().to_string())
                    .collect::<String>()
                    .trim_end()
                    .to_string()
            })
            .collect()
    }

    fn app_with(entries: Vec<Entry>) -> App {
        let mut app = App::new();
        app.status = Status::Connected {
            state: "ready".to_string(),
            message: "Printer is ready".to_string(),
        };
        // `push` leaves the offset alone, so a fresh app is already at the
        // bottom and every entry is in view.
        for entry in entries {
            app.push(entry);
        }
        app
    }

    // -----------------------------------------------------------------------
    // Input
    // -----------------------------------------------------------------------

    #[test]
    fn test_editing_at_the_cursor() {
        let mut input = Input::default();
        for character in "helo".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        // The cursor is at the end; step back over the `l` and put it back.
        input.edit(KeyCode::Left, false);
        input.edit(KeyCode::Char('l'), false);
        assert_eq!(input.text(), "hello");

        input.edit(KeyCode::Home, false);
        input.edit(KeyCode::Delete, false);
        assert_eq!(input.text(), "ello");
        input.edit(KeyCode::End, false);
        input.edit(KeyCode::Backspace, false);
        assert_eq!(input.text(), "ell");
    }

    #[test]
    fn test_ctrl_a_and_ctrl_e_move_the_cursor() {
        let mut input = Input::default();
        for character in "abc".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        input.edit(KeyCode::Char('a'), true);
        input.edit(KeyCode::Char('X'), false);
        assert_eq!(input.text(), "Xabc");

        input.edit(KeyCode::Char('e'), true);
        input.edit(KeyCode::Char('Y'), false);
        assert_eq!(input.text(), "XabcY");
    }

    #[test]
    fn test_ctrl_u_abandons_the_line_without_remembering_it() {
        let mut input = Input::default();
        for character in "half typed".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        input.edit(KeyCode::Char('u'), true);
        assert!(input.is_empty());
        // Nothing went into history, so there is nothing to recall.
        input.edit(KeyCode::Up, false);
        assert!(input.is_empty());
    }

    #[test]
    fn test_take_remembers_the_line_and_clears_it() {
        let mut input = Input::default();
        for character in "info".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        assert_eq!(input.take(), "info");
        assert!(input.is_empty());
        // Whitespace is not worth remembering.
        input.edit(KeyCode::Char(' '), false);
        assert_eq!(input.take(), " ");
        input.edit(KeyCode::Up, false);
        assert_eq!(input.text(), "info");
    }

    #[test]
    fn test_history_walks_back_and_forward_to_an_empty_line() {
        let mut input = Input::default();
        for line in ["first", "second"] {
            for character in line.chars() {
                input.edit(KeyCode::Char(character), false);
            }
            input.take();
        }

        input.edit(KeyCode::Up, false);
        assert_eq!(input.text(), "second");
        input.edit(KeyCode::Up, false);
        assert_eq!(input.text(), "first");
        // The oldest entry is the end of the history, not a wrap-around.
        input.edit(KeyCode::Up, false);
        assert_eq!(input.text(), "first");
        input.edit(KeyCode::Down, false);
        assert_eq!(input.text(), "second");
        // Past the newest is the line that was being typed, which is empty.
        input.edit(KeyCode::Down, false);
        assert!(input.is_empty());
    }

    #[test]
    fn test_a_long_line_scrolls_to_keep_the_cursor_visible() {
        let mut input = Input::default();
        for character in "0123456789".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        // Ten characters do not fit in four columns. The cursor needs a cell of
        // its own, so the last four columns are three characters plus the cursor
        // — which is what a shell prompt does too.
        let (shown, cursor) = input.window(4);
        assert_eq!(shown, "789");
        assert_eq!(cursor, 3);
        assert!(shown.chars().count() < 4, "the cursor needs a cell too");

        // Far enough back that the cursor is not at the end: the window ends at
        // the cursor, showing the lines it is between.
        input.edit(KeyCode::Home, false);
        input.edit(KeyCode::Right, false);
        input.edit(KeyCode::Right, false);
        let (shown, cursor) = input.window(4);
        assert_eq!(shown, "0123");
        assert_eq!(cursor, 2);
    }

    // -----------------------------------------------------------------------
    // Drawing
    // -----------------------------------------------------------------------

    #[test]
    fn test_the_header_shows_the_printer_state() {
        let rows = render(&app_with(Vec::new()), 60, 6);
        assert!(rows[0].contains("ready"), "{rows:?}");
        assert!(rows[0].contains("Printer is ready"), "{rows:?}");
    }

    #[test]
    fn test_the_log_shows_replies_pushes_and_notices() {
        let app = app_with(vec![
            Entry::notice(Notice::Info, "Connected to unix:/tmp/klippy_uds."),
            Entry::Sent {
                id: Some(2),
                method: "objects/query".to_string(),
                message: serde_json::json!({"id": 2, "method": "objects/query"}),
            },
            Entry::Push(serde_json::json!({"method": "klippy:status"})),
        ]);
        let rows = render(&app, 60, 8);
        let text = rows.join("\n");

        assert!(
            text.contains("Connected to unix:/tmp/klippy_uds."),
            "{text}"
        );
        assert!(text.contains("> id: 2"), "{text}");
        assert!(text.contains("method: objects/query"), "{text}");
        assert!(text.contains("< method: klippy:status"), "{text}");
    }

    #[test]
    fn test_an_error_reply_is_marked() {
        let reply = crate::connection::Reply {
            id: serde_json::json!(3),
            method: Some("gcode/script".to_string()),
            message: serde_json::json!({
                "id": 3,
                "error": {"error": "WebRequestError", "message": "Printer is halted"}
            }),
        };
        let rows = render(&app_with(vec![Entry::Reply(reply)]), 60, 6);
        assert!(
            rows.iter()
                .any(|row| row.contains("! 3 (gcode/script) Printer is halted")),
            "{rows:?}"
        );
    }

    #[test]
    fn test_the_hint_line_names_the_keys() {
        // Use wider width to fit the full hint text.
        let rows = render(&app_with(Vec::new()), 100, 6);
        let footer = rows.last().unwrap();
        assert!(footer.contains("Enter send"), "{footer}");
        assert!(footer.contains("^C quit"), "{footer}");
        assert!(footer.contains("Home/End"), "{footer}");
    }

    #[test]
    fn test_the_mouse_can_be_handed_back_to_the_terminal() {
        let mut app = app_with(Vec::new());
        assert!(app.mouse, "the window holds the mouse to begin with");

        // A captured mouse is the window's, so the terminal never sees a drag
        // and cannot select. `.mouse` (and `^S`) hands it over; the footer is
        // where that is said, since a log line would land under the frozen pane.
        assert!(app.window_command(".mouse"));
        assert!(!app.mouse);

        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("mouse released"), "{rows:?}");

        // Asking again changes nothing: the mouse is already the terminal's.
        assert!(app.window_command(".mouse"));
        assert!(!app.mouse);

        // A key is what takes it back.
        app.capture_mouse();
        assert!(app.mouse);
        let rows = render(&app, 60, 5);
        assert!(!rows.last().unwrap().contains("mouse released"), "{rows:?}");
    }

    #[test]
    fn test_a_released_mouse_freezes_the_view() {
        let entries: Vec<Entry> = (1..=8)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        // A two-row pane at the bottom: lines 7 and 8.
        let before = render(&app, 40, 5);
        assert!(before[1].contains("line 7"), "{before:?}");

        // Selection mode. The terminal's selection is anchored to the screen, so
        // a line arriving must not move what is on it.
        app.release_mouse();
        app.push(Entry::notice(Notice::Info, "line 9"));
        let after = render(&app, 40, 5);
        assert_eq!(before[1..3], after[1..3], "the view is frozen");
        assert_eq!(app.scroll, 1, "frozen means pinned, not following");

        // Taking the mouse back leaves the view where it is; `End` returns to
        // the newest line.
        app.capture_mouse();
        let rows = render(&app, 40, 5);
        assert_eq!(before[1..3], rows[1..3], "no jump on the way back");
        app.scroll = 0;
        let rows = render(&app, 40, 5);
        assert!(rows[2].contains("line 9"), "{rows:?}");
    }

    #[test]
    fn test_any_key_takes_the_mouse_back() {
        let mut app = app_with(Vec::new());
        app.release_mouse();
        assert!(!app.mouse);

        // A reader at the keyboard is not selecting, so the window takes the
        // mouse back — whatever the key is, and it still does its own job.
        app.mouse_for_key(KeyCode::PageUp, false);
        assert!(app.mouse);

        // `^S` is the one key that hands it straight back out.
        app.release_mouse();
        app.mouse_for_key(KeyCode::Char('s'), true);
        assert!(!app.mouse);

        // And any key after that takes it again.
        app.mouse_for_key(KeyCode::Enter, false);
        assert!(app.mouse);
    }

    #[test]
    fn test_scrolling_back_keeps_the_view_and_says_so() {
        let entries: Vec<Entry> = (1..=8)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        // Five rows: header, two rows of log, input, hint. Eight one-line
        // entries in a two-row pane: at the bottom we see lines 7 and 8.
        assert_eq!(app.scroll, 0, "a fresh log follows the newest line");
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("line 7"), "{rows:?}");
        assert!(rows[2].contains("line 8"), "{rows:?}");

        // Scrolled back six lines: the pane ends at line 2, so it shows the
        // oldest lines, 1 and 2.
        app.scroll = 6;
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("line 1"), "{rows:?}");
        assert!(rows[2].contains("line 2"), "{rows:?}");
        assert!(
            rows.last().unwrap().contains("viewing older entries"),
            "{rows:?}"
        );

        // Scrolling past the end shows the top, not an empty pane.
        app.scroll = usize::MAX;
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("line 1"), "{rows:?}");
        assert!(rows[2].contains("line 2"), "{rows:?}");
    }

    #[test]
    fn test_a_wrapped_entry_scrolls_a_line_at_a_time() {
        // One entry, three rows tall, in a two-row pane.
        let mut app = app_with(vec![Entry::notice(Notice::Info, "one\ntwo\nthree")]);
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("two"), "{rows:?}");
        assert!(rows[2].contains("three"), "{rows:?}");

        // One line of scroll, not one whole entry: the top becomes reachable.
        app.scroll = 1;
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("one"), "{rows:?}");
        assert!(rows[2].contains("two"), "{rows:?}");

        // And past the top is still the top.
        app.scroll = usize::MAX;
        let rows = render(&app, 40, 5);
        assert!(rows[1].contains("one"), "{rows:?}");
    }

    #[test]
    fn test_a_new_entry_returns_the_view_to_the_bottom() {
        let mut app = app_with(vec![Entry::notice(Notice::Info, "line 1")]);
        // At the bottom, a new entry is worth looking at.
        app.push(Entry::notice(Notice::Info, "line 2"));
        assert_eq!(app.scroll, 0, "a new entry is worth looking at");
    }

    #[test]
    fn test_a_new_entry_does_not_move_a_scrolled_back_view() {
        let entries: Vec<Entry> = (1..=8)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        app.scroll = 6;
        // The first draw is also what tells the window how wide the pane is.
        let before = render(&app, 40, 5);
        assert!(before[1].contains("line 1"), "{before:?}");
        assert!(before[2].contains("line 2"), "{before:?}");

        app.push(Entry::notice(Notice::Info, "line 9"));
        assert_eq!(app.scroll, 7, "the new line is counted below the viewport");
        let after = render(&app, 40, 5);
        assert_eq!(before[1..3], after[1..3], "a scrolled-back view is pinned");

        // A wrapped entry adds as many lines as it takes on screen.
        app.push(Entry::notice(Notice::Info, "a\nb\nc"));
        assert_eq!(app.scroll, 10, "three more lines landed below the viewport");
    }

    #[test]
    fn test_the_log_height_counts_wrapped_lines() {
        // A fresh app measures at width 1, so the first query has to remeasure.
        let mut app = app_with(vec![Entry::notice(Notice::Info, "one\ntwo\nthree")]);
        assert_eq!(app.total_lines(40), 3);

        app.push(Entry::notice(Notice::Info, "four"));
        assert_eq!(app.total_lines(40), 4, "one more entry, one more line");

        // The same log is taller in a narrower pane, and the scrollbar must not
        // keep reporting the width it was measured at before.
        assert_eq!(app.total_lines(4), 5, "'three' wraps into two rows at 4");

        // Clearing the log clears the measured heights with it.
        app.entries.clear();
        assert_eq!(app.total_lines(40), 0, "a cleared log has no height");
        app.push(Entry::notice(Notice::Info, "fresh"));
        assert_eq!(app.total_lines(40), 1, "and it measures again from there");
    }

    #[test]
    fn test_the_scrollbar_is_absent_when_the_log_fits() {
        let app = app_with(vec![Entry::notice(Notice::Info, "one line")]);
        assert_eq!(
            log_gutter(&app, 40, 5),
            vec![" ", " "],
            "a log that fits leaves the gutter empty"
        );
    }

    #[test]
    fn test_the_scrollbar_follows_the_scroll_position() {
        let entries: Vec<Entry> = (1..=20)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);

        // A fresh log follows the newest line, so the thumb sits at the bottom.
        let bottom = log_gutter(&app, 40, 8);
        assert_eq!(bottom.len(), 5, "the log pane's own rows");
        assert_eq!(bottom.last().unwrap(), "█", "{bottom:?}");
        assert!(bottom[..4].iter().all(|cell| cell == "│"), "{bottom:?}");

        // Scrolling back moves the thumb up the track.
        app.scroll = 8;
        let back = log_gutter(&app, 40, 8);
        assert_eq!(back[2], "█", "{back:?}");

        // Home puts it at the top, and there it stays however far past the top
        // the offset is.
        app.scroll = usize::MAX;
        let top = log_gutter(&app, 40, 8);
        assert_eq!(top[0], "█", "{top:?}");
        assert!(top[1..].iter().all(|cell| cell == "│"), "{top:?}");
    }

    #[test]
    fn test_the_arrows_scroll_once_the_log_is_scrolled_back() {
        let mut app = app_with(vec![Entry::notice(Notice::Info, "line")]);
        app.viewport.set(3);

        // At the bottom the arrows are the history's, so they fall through.
        assert!(!app.scroll_key(KeyCode::Up, false));
        assert!(!app.scroll_key(KeyCode::Down, false));
        assert_eq!(app.scroll, 0);

        // Ctrl+↑ scrolls a line from the bottom, the way it always has.
        assert!(app.scroll_key(KeyCode::Up, true));
        assert_eq!(app.scroll, 1);

        // Scrolled back, the plain arrows move the log a line at a time.
        assert!(app.scroll_key(KeyCode::Up, false));
        assert_eq!(app.scroll, 2);
        assert!(app.scroll_key(KeyCode::Down, false));
        assert_eq!(app.scroll, 1);
        assert!(app.scroll_key(KeyCode::Down, false));
        assert_eq!(app.scroll, 0, "Down returns to the bottom");

        // And back at the bottom they are the history's again.
        assert!(!app.scroll_key(KeyCode::Down, false));

        // The paging keys stay the log's whatever the offset.
        assert!(app.scroll_key(KeyCode::PageUp, false));
        assert_eq!(app.scroll, 3, "a page is the pane's height");
        assert!(app.scroll_key(KeyCode::End, false));
        assert_eq!(app.scroll, 0);
        assert!(app.scroll_key(KeyCode::Home, false));
        assert_eq!(app.scroll, usize::MAX);

        // Anything else is the input line's.
        assert!(!app.scroll_key(KeyCode::Char('x'), false));
        assert!(!app.scroll_key(KeyCode::Left, false));
    }

    #[test]
    fn test_the_scrollbar_is_a_mouse_target() {
        let entries: Vec<Entry> = (1..=20)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        // The first draw is what puts the bar (and its column) on screen.
        let _ = render(&app, 40, 8);
        let gutter = app.gutter.get();
        assert_eq!(gutter.width, 1, "the bar owns the last column");

        // The pointer has to be in the bar's own column and its own rows.
        assert_eq!(app.gutter_row(gutter.x, gutter.y), Some(0));
        assert_eq!(
            app.gutter_row(gutter.x, gutter.y + gutter.height - 1),
            Some(4)
        );
        assert_eq!(
            app.gutter_row(gutter.x - 1, gutter.y),
            None,
            "text is not a target"
        );
        assert_eq!(app.gutter_row(gutter.x, gutter.y + gutter.height), None);

        // The top of the track is the top of the log, the bottom is the newest
        // line, and the middle is in between.
        app.scroll_to_track(0);
        assert_eq!(app.scroll, 15, "the oldest lines");
        assert_eq!(log_gutter(&app, 40, 8)[0], "█", "the thumb follows");

        app.scroll_to_track(2);
        assert_eq!(app.scroll, 8, "the middle");

        app.scroll_to_track(4);
        assert_eq!(app.scroll, 0, "the newest line, and following again");
    }

    #[test]
    fn test_a_log_that_fits_has_no_mouse_target() {
        let app = app_with(vec![Entry::notice(Notice::Info, "one line")]);
        let _ = render(&app, 40, 8);
        assert_eq!(app.gutter.get().width, 0, "no bar, no target");
        assert_eq!(app.gutter_row(39, 2), None);
    }

    #[test]
    fn test_a_click_and_a_drag_on_the_bar_scroll_the_log() {
        let entries: Vec<Entry> = (1..=20)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        let _ = render(&app, 40, 8);
        let gutter = app.gutter.get();

        let click = |kind, row| MouseEvent {
            kind,
            column: gutter.x,
            row,
            modifiers: KeyModifiers::empty(),
        };

        // A press at the top of the track goes to the oldest lines.
        handle_mouse(
            &mut app,
            click(MouseEventKind::Down(MouseButton::Left), gutter.y),
        );
        assert_eq!(app.scroll, 15);

        // A drag at the bottom goes back to the newest line.
        handle_mouse(
            &mut app,
            click(
                MouseEventKind::Drag(MouseButton::Left),
                gutter.y + gutter.height - 1,
            ),
        );
        assert_eq!(app.scroll, 0);

        // A click in the text, or on the wheel's own kind, is not the bar's.
        handle_mouse(
            &mut app,
            MouseEvent {
                kind: MouseEventKind::Down(MouseButton::Left),
                column: gutter.x - 1,
                row: gutter.y,
                modifiers: KeyModifiers::empty(),
            },
        );
        assert_eq!(app.scroll, 0);
        handle_mouse(&mut app, click(MouseEventKind::ScrollUp, gutter.y));
        assert_eq!(app.scroll, 3, "the wheel still scrolls by three lines");
    }

    #[test]
    fn test_a_drag_survives_leaving_the_bar() {
        let entries: Vec<Entry> = (1..=20)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        let _ = render(&app, 40, 8);
        let gutter = app.gutter.get();

        let event = |kind, column, row| MouseEvent {
            kind,
            column,
            row,
            modifiers: KeyModifiers::empty(),
        };

        // Take hold of the bar in its middle.
        handle_mouse(
            &mut app,
            event(
                MouseEventKind::Down(MouseButton::Left),
                gutter.x,
                gutter.y + 2,
            ),
        );
        assert_eq!(app.scroll, 8, "the pointer's row picked the offset");

        // Wander left, out of the bar's column, and to the last row: the button
        // holds the drag, so it keeps following the pointer.
        handle_mouse(
            &mut app,
            event(MouseEventKind::Drag(MouseButton::Left), 0, gutter.y + 4),
        );
        assert_eq!(app.scroll, 0, "the bottom row is the bottom of the log");

        // Above the pane clamps to the top instead of getting lost.
        handle_mouse(
            &mut app,
            event(MouseEventKind::Drag(MouseButton::Left), 0, 0),
        );
        assert_eq!(app.scroll, 15);

        // Releasing lets go: a drag afterwards moves nothing.
        handle_mouse(&mut app, event(MouseEventKind::Up(MouseButton::Left), 0, 0));
        handle_mouse(
            &mut app,
            event(
                MouseEventKind::Drag(MouseButton::Left),
                gutter.x,
                gutter.y + 2,
            ),
        );
        assert_eq!(app.scroll, 15, "the button is up; the drag is over");

        // A press on the bar takes hold; a press on the text lets go.
        handle_mouse(
            &mut app,
            event(
                MouseEventKind::Down(MouseButton::Left),
                gutter.x,
                gutter.y + 2,
            ),
        );
        assert!(app.dragging);
        handle_mouse(
            &mut app,
            event(MouseEventKind::Down(MouseButton::Left), 0, gutter.y + 2),
        );
        assert!(!app.dragging, "only the bar starts a drag");
    }

    #[test]
    fn test_clicking_the_log_hands_the_mouse_back() {
        let entries: Vec<Entry> = (1..=20)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);
        // The first draw is what puts the log pane (and its bar) on screen.
        let _ = render(&app, 40, 8);
        let log = app.log.get();

        let down = |column, row| MouseEvent {
            kind: MouseEventKind::Down(MouseButton::Left),
            column,
            row,
            modifiers: KeyModifiers::empty(),
        };

        // A click on the bar is a scroll, and the mouse stays the window's.
        let bar = app.gutter.get().x;
        handle_mouse(&mut app, down(bar, log.y));
        assert!(app.mouse, "the bar is the window's business");
        assert!(app.dragging);

        // A click on the text is not: it hands the mouse to the terminal, which
        // is what selection needs.
        handle_mouse(&mut app, down(log.x + 2, log.y));
        assert!(!app.mouse, "the log's text is the terminal's");
        assert!(!app.dragging, "the bar's drag does not survive it");

        // And `^S` (the same toggle) takes it back. A click outside the log
        // pane neither scrolls nor hands it over.
        app.capture_mouse();
        assert!(app.mouse);
        handle_mouse(&mut app, down(log.x + 2, 0));
        assert!(app.mouse, "the header is not the log");
    }

    #[test]
    fn test_the_hint_says_how_far_back_the_log_is() {
        let entries: Vec<Entry> = (1..=8)
            .map(|n| Entry::notice(Notice::Info, format!("line {n}")))
            .collect();
        let mut app = app_with(entries);

        // At the bottom the hint is about typing, not about scrolling.
        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("↑↓ history"), "{rows:?}");

        // One line is worth saying: the thumb barely moves on a short track,
        // but the hint has the room to count.
        app.scroll = 1;
        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("1 line back"), "{rows:?}");

        app.scroll = 3;
        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("3 lines back"), "{rows:?}");

        // Past the top the offset is the furthest back there is — eight lines
        // in a two-row pane — not `usize::MAX`.
        app.scroll = usize::MAX;
        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("6 lines back"), "{rows:?}");
    }

    #[test]
    fn test_the_input_line_shows_the_prompt_and_the_cursor() {
        let mut app = app_with(Vec::new());
        for character in "list_endpoints".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
        let rows = render(&app, 40, 5);
        assert!(rows[3].starts_with("klippy> list_endpoints"), "{rows:?}");

        // A local command is marked as one before it is sent.
        app.input.edit(KeyCode::Char('u'), true);
        for character in ".help".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
        let rows = render(&app, 40, 5);
        assert!(rows[3].starts_with("local> .help"), "{rows:?}");
    }

    #[test]
    fn test_gcode_mode_shows_its_prompt_and_toggles_back() {
        let mut app = app_with(Vec::new());
        app.toggle_gcode();
        for character in "SET_PIN PIN=fan VALUE=1".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }

        let rows = render(&app, 60, 5);
        assert!(rows[3].starts_with("gcode> "), "{rows:?}");
        assert!(rows.last().unwrap().contains("g-code mode"), "{rows:?}");

        // Back to request mode.
        app.toggle_gcode();
        let rows = render(&app, 60, 5);
        assert!(rows[3].starts_with("klippy> "), "{rows:?}");

        // A local command keeps its own prompt even in g-code mode.
        app.toggle_gcode();
        app.input.edit(KeyCode::Char('u'), true);
        for character in ".help".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
        let rows = render(&app, 60, 5);
        assert!(rows[3].starts_with("local> .help"), "{rows:?}");
    }

    #[test]
    fn test_a_message_alternates_between_two_colours() {
        let sent = Entry::Sent {
            id: Some(1),
            method: "echo".to_string(),
            message: serde_json::json!({}),
        };
        let ok = crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("echo".to_string()),
            message: serde_json::json!({"id": 1, "result": {}}),
        };
        let failed = crate::connection::Reply {
            id: serde_json::json!(2),
            method: Some("echo".to_string()),
            message: serde_json::json!({
                "id": 2,
                "error": {"error": "WebRequestError", "message": "no"}
            }),
        };

        // One colour, the other, and back again.
        assert_eq!(entry_style(&sent, 0).fg, Some(MESSAGE_COLORS[0]));
        assert_eq!(
            entry_style(&Entry::Push(serde_json::json!({})), 1).fg,
            Some(MESSAGE_COLORS[1])
        );
        assert_eq!(
            entry_style(&Entry::Reply(ok), 2).fg,
            Some(MESSAGE_COLORS[0])
        );

        // A failed request is red wherever it falls in the alternation.
        assert_eq!(
            entry_style(&Entry::Reply(failed.clone()), 0).fg,
            Some(Color::Red)
        );
        assert_eq!(entry_style(&Entry::Reply(failed), 1).fg, Some(Color::Red));

        // The window's own lines keep their colours.
        assert_eq!(
            entry_style(&Entry::notice(Notice::Problem, "x"), 0).fg,
            Some(Color::Red)
        );
    }

    #[test]
    fn test_a_lost_connection_is_shown_in_the_header() {
        let mut app = app_with(Vec::new());
        app.status = Status::Closed("the API server closed the connection".to_string());
        let rows = render(&app, 60, 6);
        assert!(rows[0].contains("disconnected"), "{rows:?}");
    }

    #[test]
    fn test_wrapping_counts_characters() {
        assert_eq!(wrap("abcd", 2), vec!["ab", "cd"]);
        assert_eq!(wrap("ab\ncd", 4), vec!["ab", "cd"]);
        assert_eq!(wrap("", 4), vec![""]);
        // Short lines are untouched, long ones are not lost.
        assert_eq!(wrap("abc", 10), vec!["abc"]);
    }

    /// The log pane's lines as plain text.
    fn log_text(app: &App, width: usize, height: usize) -> Vec<String> {
        let total = app.total_lines(width);
        visible_lines(app, width, height, total)
            .lines
            .iter()
            .map(|line| {
                line.spans
                    .iter()
                    .map(|span| span.content.as_ref())
                    .collect::<String>()
            })
            .collect()
    }

    /// The log pane's rightmost column, one cell per row: the scrollbar gutter.
    fn scrollbar_column(app: &App, width: u16, height: u16) -> Vec<String> {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(|frame| draw(frame, app)).unwrap();
        let buffer = terminal.backend().buffer().clone();
        (0..height)
            .map(|row| buffer[(width - 1, row)].symbol().to_string())
            .collect()
    }

    /// The gutter rows that belong to the log pane: everything between the
    /// header row and the two rows the input line and the footer take.
    fn log_gutter(app: &App, width: u16, height: u16) -> Vec<String> {
        scrollbar_column(app, width, height)[1..(height as usize - 2)].to_vec()
    }

    #[test]
    fn test_a_long_entry_wraps_in_reading_order() {
        // The continuation comes *after* the first part, which is what
        // collecting the pane bottom-up has to preserve.
        let app = app_with(vec![Entry::notice(Notice::Info, "abcdefghij")]);

        let lines = log_text(&app, 4, 4);
        // Filter out empty lines (padding) and check the wrapped content.
        let non_empty: Vec<&str> = lines
            .iter()
            .filter(|l| !l.is_empty())
            .map(|s| s.as_str())
            .collect();
        assert_eq!(non_empty, vec!["abcd", "efgh", "ij"]);
    }

    #[test]
    fn test_a_multi_line_entry_keeps_its_line_order() {
        let app = app_with(vec![Entry::notice(Notice::Info, "one\ntwo\nthree")]);

        let lines = log_text(&app, 10, 4);
        let non_empty: Vec<&str> = lines
            .iter()
            .filter(|l| !l.is_empty())
            .map(|s| s.as_str())
            .collect();
        assert_eq!(non_empty, vec!["one", "two", "three"]);
    }

    #[test]
    fn test_the_newest_entry_is_at_the_bottom() {
        let app = app_with(vec![
            Entry::notice(Notice::Info, "first"),
            Entry::notice(Notice::Info, "second"),
        ]);

        let lines = log_text(&app, 10, 4);
        let non_empty: Vec<&str> = lines
            .iter()
            .filter(|l| !l.is_empty())
            .map(|s| s.as_str())
            .collect();
        assert_eq!(non_empty, vec!["first", "second"]);
    }

    /// A reply with a known shape.
    fn query_reply() -> crate::connection::Reply {
        crate::connection::Reply {
            id: serde_json::json!(2),
            method: Some("objects/query".to_string()),
            message: serde_json::json!({
                "id": 2,
                "result": {"eventtime": 1.5, "status": {"mcu": {"mcu_version": "abc"}}}
            }),
        }
    }

    #[test]
    fn test_a_reply_is_shown_as_yaml_behind_a_marker() {
        let text = entry_text(&Entry::Reply(query_reply()), Format::Yaml);

        assert!(text.starts_with("< id: 2\n  result:\n"), "{text}");
        assert!(text.contains("eventtime: 1.5"), "{text}");
        assert!(text.contains("mcu_version: abc"), "{text}");
        // Not the compact JSON the wire carries.
        assert!(!text.contains("{\"eventtime\""), "{text}");
    }

    #[test]
    fn test_json_keeps_the_marker_and_the_wire_form() {
        let text = entry_text(&Entry::Reply(query_reply()), Format::Json);

        assert!(text.starts_with("< {\"id\":2,"), "{text}");
        assert!(text.contains("\"mcu_version\":\"abc\""), "{text}");
        // One line: a JSON body has no continuation to indent.
        assert_eq!(text.lines().count(), 1, "{text}");
    }

    #[test]
    fn test_a_push_is_shown_as_yaml_behind_a_marker() {
        let text = entry_text(
            &Entry::Push(serde_json::json!({
                "method": "klippy:status",
                "params": {"state": "ready"}
            })),
            Format::Yaml,
        );

        assert!(
            text.starts_with("< method: klippy:status\n  params:\n"),
            "{text}"
        );
        assert!(text.contains("state: ready"), "{text}");
    }

    #[test]
    fn test_a_sent_entry_is_shown_behind_a_marker() {
        let entry = Entry::Sent {
            id: Some(2),
            method: "objects/query".to_string(),
            message: serde_json::json!({"id": 2, "method": "objects/query"}),
        };

        assert_eq!(
            entry_text(&entry, Format::Yaml),
            "> id: 2\n  method: objects/query"
        );
        assert_eq!(
            entry_text(&entry, Format::Json),
            "> {\"id\":2,\"method\":\"objects/query\"}"
        );
    }

    #[test]
    fn test_an_error_reply_stays_a_sentence() {
        let reply = crate::connection::Reply {
            id: serde_json::json!(3),
            method: Some("gcode/script".to_string()),
            message: serde_json::json!({
                "id": 3,
                "error": {"error": "WebRequestError", "message": "Printer is halted"}
            }),
        };

        for format in [Format::Yaml, Format::Json] {
            assert_eq!(
                entry_text(&Entry::Reply(reply.clone()), format),
                "! 3 (gcode/script) Printer is halted"
            );
        }
    }

    #[test]
    fn test_the_format_command_switches_and_says_so() {
        let mut app = app_with(Vec::new());
        assert_eq!(app.format, Format::Yaml, "YAML is the default");

        assert!(app.window_command(".json"));
        assert_eq!(app.format, Format::Json);
        assert!(app.window_command(".yaml"));
        assert_eq!(app.format, Format::Yaml);
        assert!(
            !app.window_command(".subscribe"),
            "the session's own command"
        );
        assert!(
            app.entries
                .iter()
                .any(|entry| entry.text().contains("message bodies are now JSON")),
            "the switch is confirmed in the log"
        );
    }

    #[test]
    fn test_two_messages_in_a_row_get_different_colours() {
        // A log line between them must not break the alternation: it counts
        // messages, not entries.
        let app = app_with(vec![
            Entry::Push(serde_json::json!({"method": "a"})),
            Entry::notice(Notice::Info, "a log line between them"),
            Entry::Push(serde_json::json!({"method": "b"})),
        ]);

        let total = app.total_lines(40);
        let lines = visible_lines(&app, 40, 8, total).lines;
        let colours: Vec<Option<Color>> = lines
            .iter()
            .filter(|line| line.spans[0].content.starts_with('<'))
            .map(|line| line.spans[0].style.fg)
            .collect();

        assert_eq!(colours.len(), 2, "{lines:?}");
        assert_ne!(colours[0], colours[1], "{lines:?}");
    }

    #[test]
    fn test_the_message_colours_reach_the_rendered_cells() {
        // The pane's `Line` styles are one thing; what the terminal is told is
        // another. This checks the second: the cells of the two message rows
        // must carry the two colours.
        let app = app_with(vec![
            Entry::Push(serde_json::json!({"method": "a"})),
            Entry::Push(serde_json::json!({"method": "b"})),
        ]);

        let mut terminal = Terminal::new(TestBackend::new(20, 8)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let buffer = terminal.backend().buffer();

        let colours: Vec<Color> = (0..8)
            .filter(|row| buffer[(0, *row)].symbol() == "<")
            .map(|row| buffer[(0, row)].fg)
            .collect();

        assert_eq!(colours.len(), 2, "{colours:?}");
        assert_eq!(colours[0], MESSAGE_COLORS[0]);
        assert_eq!(colours[1], MESSAGE_COLORS[1]);
    }

    /// The header's lines as plain text.
    fn header_text(app: &App, width: usize) -> Vec<String> {
        header_lines(app, width)
            .iter()
            .map(|line| {
                line.spans
                    .iter()
                    .map(|span| span.content.as_ref())
                    .collect::<String>()
            })
            .collect()
    }

    #[test]
    fn test_a_long_header_wraps_and_keeps_the_whole_message() {
        let mut app = app_with(Vec::new());
        app.status = Status::Connected {
            state: "shutdown".to_string(),
            message: "Internal: Section 'board_pins arduino-standard' is not valid".to_string(),
        };

        let lines = header_text(&app, 20);
        let joined: String = lines
            .iter()
            .map(|line| line.trim_start())
            .collect::<Vec<_>>()
            .join("");

        assert!(lines.len() > 1, "the header did not wrap: {lines:?}");
        assert_eq!(
            joined,
            "▲ shutdown · Internal: Section 'board_pins arduino-standard' is not valid"
        );
    }

    #[test]
    fn test_the_level_colours_match_the_hosts_own() {
        let style = |level| {
            entry_style(
                &Entry::Log {
                    level,
                    text: String::new(),
                },
                0,
            )
        };

        // The same five hues `tracing` paints levels with on a terminal.
        assert_eq!(style(LogLevel::Trace).fg, Some(Color::Magenta));
        assert_eq!(style(LogLevel::Debug).fg, Some(Color::Blue));
        assert_eq!(style(LogLevel::Info).fg, Some(Color::Green));
        assert_eq!(style(LogLevel::Warn).fg, Some(Color::Yellow));
        assert_eq!(style(LogLevel::Error).fg, Some(Color::Red));

        // A message never takes one of them, so a log line and a message are
        // told apart by more than their markers.
        for level in [
            LogLevel::Trace,
            LogLevel::Debug,
            LogLevel::Info,
            LogLevel::Warn,
            LogLevel::Error,
        ] {
            assert!(
                !MESSAGE_COLORS.contains(&style(level).fg.unwrap()),
                "{level:?} shares a colour with the messages"
            );
        }
    }

    #[test]
    fn test_only_the_level_tag_of_a_log_line_is_coloured() {
        let entry = Entry::Log {
            level: LogLevel::Info,
            text: "API server listening".to_string(),
        };
        let style = entry_style(&entry, 0);

        // The tag `entry_text` wrote, then the line itself, plain.
        let line = entry_line(
            &entry,
            "INFO  API server listening".to_string(),
            true,
            style,
        );
        assert_eq!(line.spans.len(), 2, "{line:?}");
        assert_eq!(line.spans[0].content.as_ref(), "INFO  ");
        assert_eq!(line.spans[0].style.fg, Some(Color::Green));
        assert_eq!(line.spans[1].content.as_ref(), "API server listening");
        assert_eq!(line.spans[1].style, Style::default(), "plain on purpose");

        // A wrapped continuation has no tag left to colour.
        let more = entry_line(&entry, "and more".to_string(), false, style);
        assert_eq!(more.spans.len(), 1);
        assert_eq!(more.spans[0].style, Style::default());

        // A message is styled whole: its colour is the only thing marking it.
        let message = Entry::Push(serde_json::json!({"method": "a"}));
        let line = entry_line(
            &message,
            "< method: a".to_string(),
            true,
            entry_style(&message, 0),
        );
        assert_eq!(line.spans.len(), 1);
        assert_eq!(line.spans[0].style.fg, Some(MESSAGE_COLORS[0]));
    }

    #[test]
    fn test_the_rendered_log_line_leaves_the_message_plain() {
        let app = app_with(vec![Entry::Log {
            level: LogLevel::Info,
            text: "hello".to_string(),
        }]);
        let mut terminal = Terminal::new(TestBackend::new(40, 6)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let buffer = terminal.backend().buffer();

        // A short log sits at the top of its pane: header row, then the log.
        let row = 1;
        assert_eq!(buffer[(0, row)].symbol(), "I");
        assert_eq!(buffer[(0, row)].fg, Color::Green, "the tag is coloured");
        assert_eq!(buffer[(4, row)].fg, Color::Green, "...all of it");
        assert_eq!(buffer[(6, row)].symbol(), "h", "the message starts here");
        assert_eq!(
            buffer[(6, row)].fg,
            Color::Reset,
            "and the message itself is not"
        );
    }
}
