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
//! Enter send · ↑↓ history · PgUp/PgDn · Home/End (log) · ^G g-code · .help · ^C quit
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
//! - **PgUp/PgDn**: scroll by a page
//! - **Home**: jump to the top (oldest lines)
//! - **End**: jump to the bottom (newest line)
//! - When scrolled back, new entries do not move the view — your place is kept
//!   until you return to the bottom.
//!
//! The unit is a rendered line, not a logged entry: a single entry wrapped over
//! several rows can be read a line at a time, and one keystroke moves what the
//! eye counts, not what the protocol happened to delimit.
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
//! * No text selection, no copy/paste handling beyond what the terminal does
//!   with the alternate screen.
//! * No reconnection: the window closes when the server goes away, after
//!   printing why. Reconnecting would mean re-establishing every subscription.
//! * The log keeps everything for the life of the session; a very chatty
//!   subscription will grow it without bound.

use std::cell::Cell;
use std::io::IsTerminal as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ratatui::crossterm::event::{
    self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers, MouseEvent, MouseEventKind,
};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::Paragraph;
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
    // Enable mouse capture for wheel scrolling
    let _mouse = ratatui::crossterm::event::EnableMouseCapture;
    ratatui::crossterm::execute!(std::io::stdout(), _mouse).ok();
    let outcome = event_loop(session, terminal, host_log).await;
    outcome
}

/// Gives the terminal back when the window ends, however it ends.
///
/// `ratatui::init` installs a panic hook that restores the terminal, but nothing
/// restores it when the future is simply dropped — which is what happens when
/// the host shuts down while the window is up. A guard does, because dropping
/// the future drops its locals.
struct TerminalGuard;

impl TerminalGuard {
    /// Take the terminal, restoring it when the returned guard is dropped.
    fn take() -> Self {
        Self
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Format {
    /// Block YAML: a tree with no quoting, the default.
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
            status: Status::Unknown,
            quit: false,
            gcode: false,
            gcode_subscribed: false,
            greeted: false,
            format: Format::Yaml,
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
        let added = if self.scroll > 0 {
            self.lines_of(&entry)
        } else {
            0
        };
        self.entries.push(entry);
        if self.entries.len() > LOG_LIMIT {
            self.entries.drain(..self.entries.len() - LOG_LIMIT);
        }
        self.scroll = self.scroll.saturating_add(added);
    }

    /// How many rendered lines an entry occupies at the pane's last width.
    ///
    /// The renderer lays text out exactly this way, so the count keeps a pinned
    /// viewport over the same lines when new output arrives.
    fn lines_of(&self, entry: &Entry) -> usize {
        let width = self.width.get().max(1);
        wrap(&entry_text(entry, self.format), width).len()
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
                        "{}\n\nWindow:\n  .yaml / .json   show message bodies as YAML or JSON\n  .gcode          toggle g-code mode (^G): typed lines go to gcode/script",
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
    let outcome = loop {
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
        // own output would otherwise push everything else away.
        (KeyCode::PageUp, _) => {
            // Scroll up = view older = add a page of lines.
            app.scroll = app.scroll.saturating_add(app.viewport.get().max(1));
        }
        (KeyCode::PageDown, _) => {
            // Scroll down = view newer = drop a page of lines.
            app.scroll = app.scroll.saturating_sub(app.viewport.get().max(1));
        }
        (KeyCode::Home, _) => {
            // Top = oldest. The renderer clamps this to the top of the log, so
            // it fills the pane with the oldest lines rather than leaving it
            // empty (which is what an unclamped offset would do).
            app.scroll = usize::MAX;
        }
        (KeyCode::End, _) => {
            // Bottom = newest.
            app.scroll = 0;
        }
        (KeyCode::Up, true) => {
            app.scroll = app.scroll.saturating_add(1);
        }
        (KeyCode::Down, true) => {
            app.scroll = app.scroll.saturating_sub(1);
        }
        (code, _) => app.input.edit(code, ctrl),
    }
    Ok(Control::Continue)
}

/// Handle mouse events (wheel scrolling).
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
    // Remember the pane's shape: PgUp/PgDn move a page, and a pinned viewport
    // counts new lines at this width.
    app.viewport.set(area.height as usize);
    app.width.set(area.width as usize);
    let lines = visible_lines(app, area.width as usize, area.height as usize);
    frame.render_widget(Paragraph::new(Text::from(lines)), area);
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
    let is_at_bottom = app.scroll == 0;
    let hint = if app.quit {
        "leaving…".to_string()
    } else if !is_at_bottom {
        "viewing older entries · End bottom · Home top · PgDn return · ^C quit".to_string()
    } else if app.gcode {
        "g-code mode · Enter send · ^G request mode · .gcode · ^C quit".to_string()
    } else {
        "Enter send · ↑↓ history · PgUp/PgDn · Home/End log · ^G g-code · .help · ^C quit"
            .to_string()
    };
    frame.render_widget(
        Paragraph::new(Line::from(hint)).style(Style::new().add_modifier(Modifier::DIM)),
        area,
    );
}

/// The lines the log pane shows, oldest first.
///
/// `scroll` is how many rendered lines the pane is scrolled back from the
/// bottom (0 = the newest line). The pane shows `height` lines ending there,
/// clamped to the top of the log, so scrolling past the end shows the oldest
/// lines rather than an empty pane.
fn visible_lines(app: &App, width: usize, height: usize) -> Vec<Line<'static>> {
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
        for line in wrap(&entry_text(entry, app.format), width)
            .into_iter()
            .rev()
        {
            lines.push(Line::from(Span::styled(line, style)));
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
    lines.reverse();
    lines
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
            LogLevel::Debug => Style::new().add_modifier(Modifier::DIM),
            LogLevel::Info => Style::new().fg(Color::DarkGray),
            LogLevel::Warn => Style::new().fg(Color::Yellow),
            LogLevel::Error => Style::new().fg(Color::Red),
        },
        Entry::Notice { kind, .. } => match kind {
            Notice::Info => Style::new().fg(Color::Cyan),
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
const MESSAGE_COLORS: [Color; 2] = [Color::LightYellow, Color::LightGreen];

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
        visible_lines(app, width, height)
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

        let lines = visible_lines(&app, 40, 8);
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
}
