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
//! Enter send · Tab complete · ↑↓ history · PgUp/PgDn log · ^G g-code · Esc×3 stop · /help · ^C quit
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
//! - **^S** (or `/mouse`, or a click on the log's text): hand the mouse back to
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
//! ## Completion
//!
//! `Tab` completes the word at the caret — the names of things, which is what a
//! terminal can usefully finish — and the candidates come from the places the
//! window knows names from:
//!
//! - A word starting with `/` completes from the window's own commands and the
//!   session's (`/help`, `/gcode`, `/subscribe`, …): a local command never
//!   reaches the printer, so the client is the only one who can name it.
//! - The first word of a g-code line completes from the printer's command
//!   names, as the `gcode` object of `objects/query` lists them. That table is
//!   asked for the first time g-code mode is entered and asked again on later
//!   visits until the printer answers: the state the header shows only moves
//!   when the printer reports a change (see [`event_loop`]), so a printer that
//!   has not reported `ready` yet is one there is nothing to wait for, and a
//!   printer that is not up yet refuses the query rather than making it wait,
//!   so there is nothing to lose by asking. Once answered — even with an empty
//!   list — it is kept, as the list does not change under a running session.
//! - A later word of a g-code line, before its `=`, completes from that
//!   command's parameter names, which come from the same answer. What follows
//!   the `=` is a value, and a value is the printer's business: the window has
//!   no list of those. A `/`-line's arguments are the session's, and a
//!   request's are the printer's, so neither is completed either.
//!
//! Until the printer answers, both the command names and the parameters come
//! from the built-in table [`gcode_params::BUILTIN`] — generated from this
//! host's sources, so it is the right list for a printer of this host — and the
//! first `Tab` that draws on it says so once in the log.
//!
//! One candidate the line does not already spell is simply filled in; one the
//! line already spells opens the layer on it, so that a `Tab` on a whole name
//! shows something rather than replacing the word with itself. Several narrow
//! the line to what all of them agree on and open a layer above the input line,
//! where `Tab`/`BackTab` walk the candidates and `Backspace` or any other key
//! closes the layer and does its own job — a key that only closed a layer would
//! be a keypress thrown away.
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
//!   for its wheel and its scrollbar. `^S` / `/mouse` / a click on the log gives
//!   it back, and the next key takes it again.
//! * No reconnection: the window closes when the server goes away, after
//!   printing why. Reconnecting would mean re-establishing every subscription.
//! * The log keeps everything for the life of the session; a very chatty
//!   subscription will grow it without bound.

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
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
use ratatui::widgets::{Clear, Paragraph, Scrollbar, ScrollbarOrientation, ScrollbarState};
use ratatui::Frame;
use serde_json::Value;

use klippy_api::address::ApiTarget;
use klippy_api::TransportError;

use crate::gcode_params;
use crate::session::{self, Control, Entry, LogLevel, Notice, Output, Session};

/// How often the keyboard thread wakes to check whether it should stop.
///
/// It cannot be interrupted out of `event::read`, so the thread polls instead;
/// this is the price of leaving the window instantly.
const KEY_POLL: Duration = Duration::from_millis(100);

/// How many `Esc` presses in a row stop the printer.
///
/// `Esc` still means "leave the window", so the presses have to be part of one
/// gesture: see [`ESTOP_WINDOW`].
const ESTOP_PRESSES: u8 = 3;

/// How long one `Esc` press stays part of the emergency-stop gesture.
///
/// `Esc` is the emergency stop and nothing else: a press that is not followed by
/// another within this window simply stops counting. Leaving is `^C` / `^D`.
const ESTOP_WINDOW: Duration = Duration::from_millis(800);

/// How long to keep reading after the window is closing, so replies already on
/// their way are not lost.
const LEAVE_GRACE: Duration = Duration::from_secs(1);

/// How many entries the log keeps before dropping the oldest.
const LOG_LIMIT: usize = 2_000;

/// How many candidates the input line's completion layer shows at once.
///
/// Six is what fits above an input line without eating the log the window is
/// for; a longer list scrolls (see [`Completion::first_shown`]).
const COMPLETION_ROWS: usize = 6;

/// How long the window waits for the printer's command list when g-code mode is
/// entered.
///
/// The ask is made once, when it is needed, and the window waits for it: the
/// reply is read on the same event loop that draws, so entering g-code mode can
/// pause for up to this long before the first frame of it appears. That is why
/// it is short — a printer answering at all answers in milliseconds, and one
/// that does not has no list to offer this time round.
const GCODE_COMMANDS_WINDOW: Duration = Duration::from_secs(2);

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
    /// The candidate layer, while one is open (see [the module docs](self)).
    completion: Option<Completion>,
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
    /// never sees the drag. `^S` / `/mouse` hands it back.
    mouse: bool,
    /// The connection's state, shown in the header.
    status: Status,
    /// Set by a local command that asked to leave.
    quit: bool,
    /// `Esc` presses seen in the current gesture, and when that gesture stops
    /// counting (`ESTOP_PRESSES` presses inside `ESTOP_WINDOW` = emergency stop).
    escape_streak: u8,
    escape_deadline: Option<tokio::time::Instant>,
    /// Whether typed lines are sent as G-Code (`gcode/script`).
    gcode: bool,
    /// Whether this window has already subscribed to G-Code output; every
    /// `gcode/subscribe_output` registers another output handler.
    gcode_subscribed: bool,
    /// The printer's own command names, from the `gcode` object of
    /// `objects/query`, for completing a g-code line. Empty until g-code mode is
    /// first entered (see [`App::ensure_gcode_commands`]) — and empty with
    /// [`App::gcode_commands_asked`] false is what a completion falls back to the
    /// built-in table from (see [`App::command_names`]).
    gcode_commands: Vec<String>,
    /// The parameter names the printer gave for each of those commands, from
    /// the same answer. A command it named none for is absent, and the built-in
    /// table stands in for it (see [`App::parameter_names`]).
    gcode_parameters: HashMap<String, Vec<String>>,
    /// Whether the printer's command table has been answered for this connection.
    ///
    /// Answered — even with nothing in it — is what stops the window asking
    /// again: a printer that has no named commands would otherwise be asked
    /// once per `^G`. A refusal (a printer still coming up) leaves this false
    /// on purpose, so the next visit can try again.
    ///
    /// It is also what says which table a completion drew on: until it is true
    /// the window completes from its built-in table.
    gcode_commands_asked: bool,
    /// Whether the log has been told that a completion drew on the built-in
    /// table (see [`App::warn_about_the_builtin_table`]). One per session, not
    /// one per `Tab`: it is the same fact every time, and the log is for what
    /// happened.
    gcode_fallback_warned: bool,
    /// Whether the handshake's `info` has been answered.
    ///
    /// Until it has, an `info` reply is the header's business rather than the
    /// log's — but only that one: a later, typed `info` is the user's, and it
    /// belongs in the log like any other reply.
    greeted: bool,
    /// How the window renders an entry: the body format plus whether g-code
    /// mode strips the API envelope off `gcode/script` traffic.
    render: Render,
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

/// How the window renders an entry.
///
/// The body format (YAML or JSON) is one axis; whether g-code mode strips the
/// API envelope off `gcode/script` traffic is the other. The two are kept
/// together because every rendering path needs both at once, and a change to
/// either one re-wraps the whole log — which is why [`Heights`] stores the pair
/// it was measured at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Render {
    /// How to show a message body.
    format: Format,
    /// Whether g-code mode is on, so `gcode/script` traffic shows as the bare
    /// G-Code exchange rather than the API envelope.
    gcode: bool,
}

impl Render {
    /// YAML bodies, request mode: the default view.
    const DEFAULT: Self = Self {
        format: Format::Yaml,
        gcode: false,
    };
}

impl App {
    fn new() -> Self {
        Self {
            entries: Vec::new(),
            input: Input::default(),
            completion: None,
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
            escape_streak: 0,
            escape_deadline: None,
            gcode: false,
            gcode_subscribed: false,
            gcode_commands: Vec::new(),
            gcode_parameters: HashMap::new(),
            gcode_commands_asked: false,
            gcode_fallback_warned: false,
            greeted: false,
            render: Render::DEFAULT,
        }
    }

    /// Hand the mouse to the terminal, so it can select text.
    ///
    /// The window holds the mouse — the wheel and the scrollbar need it — so
    /// letting go is the move that has to be asked for, and this is how: `^S`,
    /// `/mouse`, or a click on the log's text. Holding it again is not asked
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
    ///
    /// The render option is kept in step with [`App::gcode`] so the whole log
    /// re-wraps on the switch: in g-code mode a past `gcode/script` exchange
    /// is shown as the bare script and output, which takes a different number
    /// of lines than its envelope did.
    fn toggle_gcode(&mut self) {
        self.gcode = !self.gcode;
        self.render.gcode = self.gcode;
        let text = if self.gcode {
            "g-code mode: typed lines go to gcode/script, output shown raw (^G or /gcode to leave)"
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

    /// Whether the printer's command list still has to be asked for.
    ///
    /// Kept apart from the ask itself so the gate can be tested without a
    /// session: this is the whole of the decision. A list is only useful when
    /// g-code mode is on (that is the only thing that completes from it), and
    /// until it has been answered there is nothing to lose by asking.
    ///
    /// [`Status`] is deliberately not consulted for a `ready` state. The header
    /// starts at the handshake's `info` reply and is updated from then on only
    /// by a `webhooks` push, which says what *changed* — so until the printer
    /// reports one it still holds the connect-time value, and gating on `ready`
    /// would mean not asking during exactly the wait this is here for. Asking
    /// instead is cheap — a printer that is not up refuses `objects/query` at
    /// once rather than making it wait — and a refusal leaves the flag false, so
    /// the next `Tab` asks again.
    fn needs_gcode_commands(&self) -> bool {
        self.gcode && !self.gcode_commands_asked
    }

    /// Ask the printer for its command table, once, for completion.
    ///
    /// The `gcode` object of `objects/query` is the only list of command names,
    /// and the one place the parameter names are reported too, so one ask
    /// answers both. The printer cannot answer it until it is up, so the ask is
    /// made the first time g-code mode is entered and repeated on later visits
    /// until it is answered (see [`App::needs_gcode_commands`] for why `ready`
    /// is not waited for).
    ///
    /// Failing is not news of its own: completion falls back to the built-in
    /// table, which says what it can, and a notice per `^G` about a printer
    /// that is simply still loading would be noise in the log that the log is
    /// for.
    async fn ensure_gcode_commands(&mut self, session: &mut Session) {
        if !self.needs_gcode_commands() {
            return;
        }
        if let Ok(Some(commands)) = session.gcode_commands(self, GCODE_COMMANDS_WINDOW).await {
            self.gcode_commands = commands.names;
            self.gcode_parameters = commands.parameters;
            // A refusal is not an answer: the flag stays false so the next
            // visit to the mode can ask a printer that has come up since.
            self.gcode_commands_asked = true;
        }
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
        let height = entry_height(&entry, width, self.render);
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
            || measured.render != self.render
            || measured.heights.len() != self.entries.len()
        {
            measured.remeasure(&self.entries, width, self.render);
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
    /// the pane width, the render options, or the set of entries changed.
    fn total_lines(&self, width: usize) -> usize {
        let width = width.max(1);
        let mut measured = self.heights.borrow_mut();
        // The heights parallel the entries; if anything emptied one without the
        // other (`Ctrl+L`), the two disagree and the log is measured again.
        if measured.width != width
            || measured.render != self.render
            || measured.heights.len() != self.entries.len()
        {
            measured.remeasure(&self.entries, width, self.render);
        }
        measured.total
    }

    /// Handle a local command only the window has.
    ///
    /// Returns whether the line was one; the session's own commands
    /// (`/subscribe`, `/quit`) go to the session. `/yaml` and `/json` are the
    /// window's because the line front-end has nothing to switch — it is one
    /// compact JSON line per event by design — and `/help` is answered here so
    /// that the window's commands sit in the same list as the session's.
    fn window_command(&mut self, line: &str) -> bool {
        match line.trim() {
            "/gcode" => {
                self.toggle_gcode();
                true
            }
            "/mouse" => {
                self.release_mouse();
                true
            }
            "/yaml" | "/json" => {
                self.render.format = if line.trim() == "/json" {
                    Format::Json
                } else {
                    Format::Yaml
                };
                let name = match self.render.format {
                    Format::Yaml => "YAML",
                    Format::Json => "JSON",
                };
                self.push(Entry::notice(
                    Notice::Info,
                    format!("message bodies are now {name}"),
                ));
                true
            }
            "/help" => {
                self.push(Entry::notice(
                    Notice::Info,
                    format!(
                        "{}\n\nWindow:\n  /yaml / /json   show message bodies as YAML or JSON\n  /gcode          toggle g-code mode (^G): typed lines go to gcode/script\n  /mouse          hand the mouse back to the terminal (^S, or click the log) so text can be selected\n  Tab / BackTab   complete the word at the caret, then walk the candidates; any other key closes the list\n  Backspace       closes the candidate list and deletes, in one press\n  ^↑ / ^↓         scroll the log one line, even at the newest entry\n  Esc ×3          emergency stop (Esc on its own does nothing)",
                        session::usage()
                    ),
                ));
                true
            }
            _ => false,
        }
    }
}

/// What a press of `Esc` asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Escape {
    /// Part of a gesture: nothing happens yet, keep waiting for more presses.
    Wait,
    /// Three presses in a row: stop the printer.
    Stop,
}

impl App {
    /// Take one `Esc` press.
    ///
    /// The gesture is "three presses in a row": each press within
    /// [`ESTOP_WINDOW`] of the previous one extends the streak, and a press that
    /// arrives after the window starts a new one. A streak that never reaches
    /// [`ESTOP_PRESSES`] does nothing: `Esc` is the emergency stop here, and
    /// leaving is `^C` / `^D` (the footer says so while a streak is counting).
    fn escape(&mut self, now: tokio::time::Instant) -> Escape {
        self.escape_streak = match self.escape_deadline {
            Some(deadline) if now < deadline => self.escape_streak.saturating_add(1),
            _ => 1,
        };
        self.escape_deadline = Some(now + ESTOP_WINDOW);
        if self.escape_streak >= ESTOP_PRESSES {
            self.escape_streak = 0;
            self.escape_deadline = None;
            return Escape::Stop;
        }
        Escape::Wait
    }

    /// Retire a gesture whose window has closed.
    ///
    /// `Esc` does nothing on its own, so this only stops the footer from
    /// counting a streak the user has walked away from (the next `Esc` would
    /// start a new one anyway).
    fn escape_window_closed(&mut self, now: tokio::time::Instant) -> bool {
        match self.escape_deadline {
            Some(deadline) if now >= deadline => {
                self.escape_streak = 0;
                self.escape_deadline = None;
                true
            }
            _ => false,
        }
    }

    /// The footer text while an `Esc` gesture is in progress.
    fn escape_hint(&self, now: tokio::time::Instant) -> Option<String> {
        let pending = self.escape_streak > 0
            && self.escape_streak < ESTOP_PRESSES
            && self.escape_deadline.is_some_and(|deadline| now < deadline);
        pending.then(|| {
            format!(
                "emergency stop {}/{} · Esc again · any other key cancels · ^C quits",
                self.escape_streak, ESTOP_PRESSES
            )
        })
    }

    /// Move the header onto the printer state a `status` object reports.
    ///
    /// Both callers hand in the same shape — an `objects/…` `status` payload
    /// holding a `webhooks` object — so the reading of it lives in one place.
    /// Only a status that carries both fields is applied: the header shows a
    /// state and its message together, and half of one says less than what is
    /// already there.
    fn apply_status(&mut self, status: &Value) {
        let Some(webhooks) = status.get("webhooks") else {
            return;
        };
        let state = webhooks.get("state").and_then(Value::as_str);
        let message = webhooks.get("state_message").and_then(Value::as_str);
        if let (Some(state), Some(message)) = (state, message) {
            self.status = Status::Connected {
                state: state.to_string(),
                // The header is a single row, so a multi-line message would
                // fold it open.
                message: message.replace('\n', " "),
            };
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
            // The `objects/subscribe` reply is a snapshot of the objects as
            // they were when the subscription was taken, while the pushes that
            // follow carry only what *changes*. A state that settles in
            // between — after the handshake's `info` above and before the
            // subscription was answered — would therefore never be pushed, and
            // the header would stay on the `info` value for good. So the
            // snapshot is read too. It stays in the log like any other reply,
            // which is why this arm does not `return`.
            Entry::Reply(reply)
                if self.greeted
                    && reply.method.as_deref() == Some("objects/subscribe")
                    && !reply.is_error() =>
            {
                if let Some(status) = reply.result().and_then(|result| result.get("status")) {
                    self.apply_status(status);
                }
            }
            // After that it is the `webhooks` object that reports the state, on
            // every change — so a subscription keeps the header honest without
            // anyone asking.
            Entry::Push(message) => {
                let status = message
                    .get("params")
                    .and_then(|params| params.get("status"));
                if let Some(status) = status {
                    self.apply_status(status);
                }
            }
            _ => (),
        }
        // G-code mode strips the API envelope down to the G-Code exchange
        // itself: the `gcode/script` request and the `gcode:output` push are
        // rewritten by the renderer, and the *successful* `gcode/*` plumbing —
        // the subscription, the restarts, the script's own `{}` reply — is
        // dropped, because a notice or the output subscription already said
        // what it did. Failures are kept: see [`gcode_suppress`].
        if self.gcode && gcode_suppress(&entry) {
            return;
        }
        self.push(entry);
    }
}

/// Whether g-code mode drops `entry` from the log entirely.
///
/// The `gcode/script` request and the `gcode:output` push are kept — the
/// renderer rewrites them into the bare G-Code exchange — so they are not
/// suppressed. What is suppressed is the *successful* plumbing around them: the
/// `{}` reply to `gcode/script` (the output subscription already carried the
/// result) and the `gcode/*` requests whose effect a notice announced (the
/// subscription, the restarts).
///
/// A failure is never suppressed. A command error normally arrives twice — as a
/// `!!` line on the output subscription and as the reply's `error` — but when no
/// dispatcher is up yet, `gcode/subscribe_output` fails too, so there is no
/// subscription and the reply is the only place the error appears.
fn gcode_suppress(entry: &Entry) -> bool {
    match entry {
        // `gcode/script` is the user's line, so its request is shown; only its
        // successful reply is dropped, since the output subscription carries
        // the result.
        Entry::Sent { method, .. } => is_gcode_plumbing_method(method),
        Entry::Reply(reply) => {
            !reply.is_error()
                && reply.method.as_deref().is_some_and(|method| {
                    method == "gcode/script" || is_gcode_plumbing_method(method)
                })
        }
        _ => false,
    }
}

/// Whether `method` is a `gcode/*` endpoint other than `gcode/script`.
///
/// `gcode/script` is the one the user drives, so its request is shown (as the
/// typed script); the rest — `gcode/subscribe_output`, `gcode/restart`,
/// `gcode/firmware_restart` — is plumbing the window or a local command asked
/// for, and a notice already said what it did.
fn is_gcode_plumbing_method(method: &str) -> bool {
    method.starts_with("gcode/") && method != "gcode/script"
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
    // The header would otherwise stand still at what the handshake's `info`
    // reply said: the printer reports every later state change through its
    // `webhooks` object, and nothing pushes that unless it is subscribed to.
    session.subscribe_webhooks(&mut app).await?;

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
            /// The `Esc` streak window closed: stop counting the gesture.
            EscapeWindow,
        }
        let escape_deadline = app.escape_deadline;
        let step = tokio::select! {
            key = keys.recv() => Step::Key(key),
            message = session.receive() => Step::Message(message),
            // A host that embedded this window keeps talking about itself while
            // the printer runs; its lines go into the same log, in order.
            entry = async { host_log.as_mut()?.recv().await }, if host_log.is_some() => {
                Step::HostLog(entry)
            }
            // Pending forever when no `Esc` press is being counted, so the arm
            // costs nothing until one is.
            _ = async {
                match escape_deadline {
                    Some(deadline) => tokio::time::sleep_until(deadline).await,
                    None => std::future::pending().await,
                }
            } => Step::EscapeWindow,
        };

        match step {
            // The keyboard thread is gone, which only happens on shutdown.
            Step::Key(None) => break Ok(()),
            Step::Key(Some(Event::Key(key))) => {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                // Any other key means the gesture was not an emergency stop.
                if key.code != KeyCode::Esc {
                    app.escape_streak = 0;
                    app.escape_deadline = None;
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
            Step::EscapeWindow => {
                app.escape_window_closed(tokio::time::Instant::now());
            }
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
    // `Tab` is the last chance to ask for the printer's command list: a `Tab` on
    // a g-code line whose list was never answered has nothing to offer, so the
    // ask is repeated here. The gate closes on the first answer, so a printer
    // that has a list is only ever asked once; until it answers, a `Tab` may be
    // a round trip, which is the price of not having a live status to wait on.
    if key.code == KeyCode::Tab {
        app.ensure_gcode_commands(session).await;
    }
    // The candidate layer settles next, and this key still does its own job
    // after — except `Tab`, which is the layer's own key.
    if app.completion_key(key.code) {
        return Ok(Control::Continue);
    }
    match (key.code, ctrl) {
        // Leaving: ^C and ^D leave at once.
        (KeyCode::Char('c') | KeyCode::Char('d'), true) => return Ok(Control::Quit),
        // `Esc` is the emergency stop: three presses in a row, and nothing on
        // its own. Leaving is `^C` / `^D` (said in the footer while counting).
        (KeyCode::Esc, _) => {
            return match app.escape(tokio::time::Instant::now()) {
                Escape::Stop => {
                    session.emergency_stop(app).await?;
                    Ok(Control::Continue)
                }
                Escape::Wait => Ok(Control::Continue),
            }
        }
        (KeyCode::Char('l'), true) => {
            app.entries.clear();
            app.scroll = 0;
        }
        // Switching what a typed line means: a request, or G-Code.
        (KeyCode::Char('g'), true) => {
            app.toggle_gcode();
            app.ensure_gcode_subscription(session).await?;
            app.ensure_gcode_commands(session).await;
        }
        (KeyCode::Enter, _) => {
            let line = app.input.take();
            if line.is_empty() {
                return Ok(Control::Continue);
            }
            app.scroll = 0;
            if app.window_command(&line) {
                // Entering g-code mode is the window's business, but the output
                // subscription it needs belongs to the session — and so does
                // the command list that completing a g-code line draws from.
                app.ensure_gcode_subscription(session).await?;
                app.ensure_gcode_commands(session).await;
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
    // Last, because it is an overlay: the candidate layer is drawn over the log
    // rather than given rows of its own, and the four panes keep their sizes.
    draw_completions(frame, app, input, log);
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

/// Draw the open candidate layer just above the input line.
///
/// It hangs off the input line — same left edge, bottom row against the line
/// above it — so that it reads as belonging to the word being typed. It overlays
/// the log instead of taking rows of its own, since the panes' heights are fixed
/// in [`draw`] and the candidates are a passing thing; nothing is drawn when the
/// log has no room for it, which costs the picture and not the feature.
fn draw_completions(frame: &mut Frame, app: &App, input: Rect, log: Rect) {
    let Some(completion) = app.completion.as_ref() else {
        return;
    };
    if completion.candidates.is_empty() {
        return;
    }
    let rows = completion.candidates.len().min(COMPLETION_ROWS);
    // One blank column on each side of the longest candidate, so the names are
    // not against the edge of the layer — and never wider than the input line
    // the layer belongs to.
    let widest = completion
        .candidates
        .iter()
        .map(|candidate| candidate.chars().count())
        .max()
        .unwrap_or(0);
    let width = (widest + 2).min(input.width as usize);
    if width == 0 || input.y < log.y.saturating_add(rows as u16) {
        return;
    }
    let area = Rect::new(input.x, input.y - rows as u16, width as u16, rows as u16);

    // The log is behind the layer, and a `Paragraph` only writes the cells it
    // has text for: without this the log's own lines would show through the
    // gaps between the candidates.
    frame.render_widget(Clear, area);

    let first = completion.first_shown(rows);
    let lines: Vec<Line<'static>> = completion.candidates[first..first + rows]
        .iter()
        .enumerate()
        .map(|(offset, candidate)| {
            let picked = completion.selected == Some(first + offset);
            let mut text = format!(" {candidate}");
            // The picked row is a bar across the layer rather than an emphasised
            // word, so which one it is can be seen at a glance.
            if picked {
                let padding = width.saturating_sub(text.chars().count());
                text.push_str(&" ".repeat(padding));
            }
            let style = if picked {
                Style::new().reversed()
            } else {
                Style::new()
            };
            Line::from(Span::styled(text, style))
        })
        .collect();
    frame.render_widget(Paragraph::new(Text::from(lines)), area);
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
    } else if let Some(gesture) = app.escape_hint(tokio::time::Instant::now()) {
        gesture
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
        "g-code mode · Enter send · ^G request mode · /gcode · Esc×3 stop · ^C quit".to_string()
    } else {
        // Kept inside 100 columns (the footer is one line and truncates): `Tab
        // complete` cost the log's `Home/End` their place here, and the two
        // gestures that never fitted (^↑/^↓ line, and what Esc×3 means next to
        // plain Esc, and where the log's ends are) are spelled out in `/help`.
        "Enter send · Tab complete · ↑↓ history · PgUp/PgDn log · ^G g-code · Esc×3 stop · /help · ^C quit"
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
/// the render options change, since either one re-wraps everything.
#[derive(Default)]
struct Heights {
    /// The pane width these heights were measured at (0 = never measured).
    width: usize,
    /// The render options they were measured in: the body format plus whether
    /// g-code mode strips the `gcode/script` envelope.
    render: Render,
    /// One height per entry, parallel to `App::entries`.
    heights: Vec<usize>,
    /// The sum of `heights`.
    total: usize,
}

impl Heights {
    fn remeasure(&mut self, entries: &[Entry], width: usize, render: Render) {
        self.width = width;
        self.render = render;
        self.heights.clear();
        self.total = 0;
        for entry in entries {
            let height = entry_height(entry, width, render);
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
fn entry_height(entry: &Entry, width: usize, render: Render) -> usize {
    wrap(&entry_text(entry, render), width.max(1)).len()
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
        let style = entry_style(entry, index, app.render);
        let wrapped = wrap(&entry_text(entry, app.render), width);
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
fn entry_style(entry: &Entry, message_index: usize, render: Render) -> Style {
    // In g-code mode the exchange reads like a terminal: the window's `>`/`<`
    // markers are its only scaffolding, so the colour carries the direction —
    // the typed line cyan, the printer's output white — and a run of output
    // does not flicker between the message colours. A `!! ` line is an error
    // the printer reported, so it is red like any other failure.
    if render.gcode {
        if gcode_script_sent(entry).is_some() {
            return Style::new().fg(Color::Cyan);
        }
        if let Some(line) = gcode_output_line(entry) {
            let colour = if line.starts_with("!! ") {
                Color::Red
            } else {
                Color::White
            };
            return Style::new().fg(colour);
        }
    }
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
///
/// A stripped `gcode/script` request and a `gcode:output` push are messages like
/// any other; they count toward the colour alternation, though a g-code exchange
/// is drawn in one steady colour (see [`entry_style`]).
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
///
/// In g-code mode the `gcode/script` envelope is stripped: the typed script is
/// shown behind a `>` and the printer's output behind a `<` (the direction a
/// G-Code line travels), with the `method`/`params` scaffolding gone — so the
/// pane reads like a G-Code terminal rather than a wire dump. The printer's own
/// text is kept line for line, `// ` and `!! ` prefixes included: those are how
/// Klipper writes its output, so the window marks the direction rather than
/// rewriting the line. Only the continuation lines of a multi-line answer are
/// indented under the marker, the layout [`marked`] gives every message.
/// Other traffic keeps its envelope.
fn entry_text(entry: &Entry, render: Render) -> String {
    if render.gcode {
        if let Some(script) = gcode_script_sent(entry) {
            return marked('>', script);
        }
        if let Some(line) = gcode_output_line(entry) {
            return marked('<', line);
        }
    }
    match entry {
        Entry::Sent { message, .. } => marked('>', &body(message, render.format)),
        Entry::Reply(reply) if !reply.is_error() => {
            marked('<', &body(&reply.message, render.format))
        }
        Entry::Push(message) => marked('<', &body(message, render.format)),
        other => other.text(),
    }
}

/// The `script` parameter of a `gcode/script` request this client sent, when
/// that is what the entry is.
///
/// G-code mode shows the typed script rather than the `gcode/script` envelope,
/// so this is what picks it out of a [`Entry::Sent`].
fn gcode_script_sent(entry: &Entry) -> Option<&str> {
    let Entry::Sent { message, .. } = entry else {
        return None;
    };
    if message.get("method").and_then(Value::as_str) != Some("gcode/script") {
        return None;
    }
    message
        .get("params")
        .and_then(|p| p.get("script"))
        .and_then(Value::as_str)
}

/// The `response` line a `gcode:output` push carried, when that is what the
/// entry is.
///
/// G-code mode shows the printer's own output line rather than the push
/// envelope, so this is what picks it out of a [`Entry::Push`]. The line comes
/// back exactly as the printer wrote it; [`entry_text`] puts the window's `<`
/// marker in front of it.
fn gcode_output_line(entry: &Entry) -> Option<&str> {
    let Entry::Push(message) = entry else {
        return None;
    };
    if message.get("method").and_then(Value::as_str) != Some("gcode:output") {
        return None;
    }
    message
        .get("params")
        .and_then(|p| p.get("response"))
        .and_then(Value::as_str)
}

/// Put a `<`/`>` marker in front of a body, indenting the rest under it.
///
/// A body with nothing in it gets the marker by itself — no trailing space to
/// hide in the log.
fn marked(marker: char, body: &str) -> String {
    let mut lines = body.lines();
    let first = lines.next().unwrap_or_default();
    let mut text = if first.is_empty() {
        marker.to_string()
    } else {
        format!("{marker} {first}")
    };
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

    /// The word the caret is in, if it is in one.
    ///
    /// `None` is "there is nothing here to complete": the caret is on a blank
    /// — in the gap between two words, or in the blanks a line was started with
    /// — or the line is nothing but blanks.
    ///
    /// An empty line is the exception: it is a word of no characters, so `Tab`
    /// there lists everything that could be typed rather than nothing. That is
    /// the only way to browse the candidates when no prefix is known. Blank
    /// lines are not that case — they have a word, the reader just has not
    /// started it — so they stay `None`. A word of no characters at the *end*
    /// of a line that has one is the same case one step along: `SET_PIN PIN=fan `
    /// is asking what comes next, which is what lists that command's parameters.
    fn completion_word(&self) -> Option<Word> {
        if self.buffer.is_empty() {
            return Some(Word { start: 0, end: 0 });
        }
        // A blank under the caret is a gap, not a word: completion replaces the
        // word it is in, and there is none to replace there.
        if self.buffer[self.cursor..]
            .first()
            .is_some_and(|character| character.is_whitespace())
        {
            return None;
        }
        // The word runs back to the blank before the caret and forward to the
        // blank after it, or to the ends of the line: completion replaces the
        // whole of it, wherever in it the caret happens to be.
        let start = self.buffer[..self.cursor]
            .iter()
            .rposition(|character| character.is_whitespace())
            .map_or(0, |index| index + 1);
        let end = self.buffer[self.cursor..]
            .iter()
            .position(|character| character.is_whitespace())
            .map_or(self.buffer.len(), |offset| self.cursor + offset);
        if start == self.cursor && self.buffer[..start].iter().all(|c| c.is_whitespace()) {
            // Nothing but blanks: a word has been started, but there is nothing
            // in it that names anything.
            return None;
        }
        Some(Word { start, end })
    }

    /// The first word of the line, if it has one.
    ///
    /// Leading blanks are not part of it. What a later word is an argument *of*:
    /// `PIN=` is a parameter only if the line really starts with `SET_PIN`.
    fn first_word(&self) -> Option<String> {
        let start = self.buffer.iter().position(|c| !c.is_whitespace())?;
        let end = self.buffer[start..]
            .iter()
            .position(|c| c.is_whitespace())
            .map_or(self.buffer.len(), |offset| start + offset);
        Some(self.buffer[start..end].iter().collect())
    }

    /// The characters a [`Word`] covers.
    fn word_text(&self, word: &Word) -> String {
        self.buffer[word.start..word.end].iter().collect()
    }

    /// Put `replacement` where `word` was, and leave the caret after it.
    ///
    /// The caret goes to the end of the replacement rather than staying where it
    /// was: a completion finishes a name, and what the reader types next is the
    /// arguments that name takes, which go after it.
    ///
    /// Returns where the replacement now is.
    fn replace(&mut self, word: &Word, replacement: &str) -> Word {
        self.buffer
            .splice(word.start..word.end, replacement.chars());
        let end = word.start + replacement.chars().count();
        self.cursor = end;
        Word {
            start: word.start,
            end,
        }
    }
}

// ===========================================================================
// Completion
// ===========================================================================

/// The commands the window answers itself, without their leading slash.
///
/// The names rather than the lines, because the input line completes from this
/// list and [`App::window_command`] runs the same set: the test that every one
/// of these is answered holds the two spellings together.
const WINDOW_COMMANDS: &[&str] = &["gcode", "mouse", "yaml", "json", "help"];

/// Where one word of the input line is.
///
/// Completion is for one word at a time, so what it needs from the line is where
/// that word begins and ends. Both are character offsets, the unit [`Input`]
/// counts in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Word {
    /// Where the word starts, from the start of the line.
    start: usize,
    /// Where it ends (one past its last character).
    end: usize,
}

/// An open candidate layer: what `Tab` found, and what is picked so far.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Completion {
    /// The candidates, in the order they are shown.
    candidates: Vec<String>,
    /// The word the layer is completing, as it stands in the line now. It moves
    /// as candidates are picked, since each one replaces the last.
    word: Word,
    /// Which candidate is picked. `None` while the layer has only just opened
    /// from a multiple match: nothing is in the line but the candidates' shared
    /// prefix, so picking the first of them would be arbitrary.
    selected: Option<usize>,
}

impl Completion {
    /// The first candidate the layer draws, so that the picked one is in it.
    ///
    /// The list scrolls as little as it can: with nothing picked, or a pick
    /// near the front, it shows the candidates from the beginning rather than
    /// keeping the pick at a fixed row.
    fn first_shown(&self, rows: usize) -> usize {
        self.selected
            .unwrap_or(0)
            .saturating_sub(rows.saturating_sub(1))
    }
}

/// The longest prefix every one of `names` starts with.
///
/// This is what a multiple match leaves in the line: the characters the reader
/// would have had to type for any of the candidates, so that `Tab` narrows the
/// line exactly as far as it can without choosing.
fn common_prefix(names: &[String]) -> String {
    let mut prefix: Vec<char> = names
        .first()
        .map(|name| name.chars().collect())
        .unwrap_or_default();
    for name in names {
        let shared = prefix
            .iter()
            .zip(name.chars())
            .take_while(|(mine, theirs)| **mine == *theirs)
            .count();
        prefix.truncate(shared);
    }
    prefix.into_iter().collect()
}

/// What a `Tab` at the caret is completing.
///
/// The line says which names are wanted: the first word of a G-Code line is a
/// command name, and a later word before its `=` is a parameter of the command
/// the line starts with. Everything else — a local word's arguments, a
/// request's, a value — has no list in the window.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Target {
    /// The first word of the line: a local command, or a G-Code command name.
    Command {
        /// The word in the line, which is what a candidate replaces.
        word: Word,
        /// What is typed of it, which is what a candidate has to start with.
        prefix: String,
    },
    /// A later word of a G-Code line, before its `=`: a parameter name of
    /// `command`. The word ends at the `=` when there is one, so that replacing
    /// it leaves the value beside it alone.
    Parameter {
        /// The command whose parameters are being completed. Each word carries
        /// its command, so a completion needs no reading back through the line.
        command: String,
        /// The name part of the word.
        word: Word,
        /// What is typed of the name, up to the caret: the caret may be in the
        /// middle of it.
        prefix: String,
    },
}

impl Target {
    /// The span of the line a candidate replaces.
    fn word(&self) -> Word {
        match self {
            Target::Command { word, .. } | Target::Parameter { word, .. } => *word,
        }
    }

    /// Whether this is a local command, which the window answers itself.
    fn is_local(&self) -> bool {
        matches!(self, Target::Command { prefix, .. } if prefix.starts_with('/'))
    }
}

/// The candidates among `names` that start with `prefix`, matched without case.
///
/// G-Code is written in capitals but typed in lower case, and what goes into
/// the line is the name as its source spells it (the printer's own, or the
/// built-in table's), so that what is inserted is what runs.
fn matching(names: &[String], prefix: &str) -> Vec<String> {
    let typed = prefix.to_ascii_lowercase();
    names
        .iter()
        .filter(|name| name.to_ascii_lowercase().starts_with(&typed))
        .cloned()
        .collect()
}

/// The parameter names the built-in table declares for `command`, which is
/// spelled the way the printer spells it (capitalised).
fn builtin_parameters(command: &str) -> &'static [&'static str] {
    gcode_params::BUILTIN
        .iter()
        .find(|(name, _)| *name == command)
        .map_or(&[], |(_, parameters)| *parameters)
}

/// The local commands — and the window's own — that start with `prefix`.
///
/// Only the canonical name of each: the aliases would double every entry, and
/// `/h` finds `/help` by prefix anyway. `help` is both the window's and the
/// session's — one entry is enough, since the reader is choosing a line to type,
/// not a place for it to go.
fn local_command_candidates(prefix: &str) -> Vec<String> {
    let mut candidates: Vec<String> = Vec::new();
    for name in session::LOCAL_COMMANDS
        .iter()
        .map(|(name, _)| *name)
        .chain(WINDOW_COMMANDS.iter().copied())
    {
        let candidate = format!("/{name}");
        if candidate.starts_with(prefix) && !candidates.contains(&candidate) {
            candidates.push(candidate);
        }
    }
    candidates.sort();
    candidates
}

impl App {
    /// The keypresses that belong to completion, and whether they were one.
    ///
    /// The whole of the feature's keyboard contract is here, away from
    /// [`handle_key`], because none of it needs a session — and a `Tab` that
    /// only works with a printer attached is a `Tab` that is never tested.
    ///
    /// `Tab` opens the layer or walks it; `BackTab` walks it back. Every other
    /// key closes the layer and is then nobody's: the caller goes on to do what
    /// it always did, which is what keeps `Backspace`, `Esc`, `Enter` and the
    /// rest exactly as they were. (`Tab` on a line with nothing to complete is
    /// still consumed: there is no tab character in a line the printer is sent.)
    fn completion_key(&mut self, code: KeyCode) -> bool {
        match code {
            KeyCode::Tab => {
                if self.completion.is_some() {
                    self.cycle_completion(1);
                } else {
                    self.complete();
                }
                true
            }
            KeyCode::BackTab if self.completion.is_some() => {
                self.cycle_completion(-1);
                true
            }
            // Nothing to walk: `BackTab` is not a way to open the layer (`Tab`
            // is), so the key stays nobody's and falls through.
            KeyCode::BackTab => false,
            _ => {
                self.completion = None;
                false
            }
        }
    }

    /// Complete the word at the caret, and open the layer if it needs choosing.
    ///
    /// A candidate the line spells differently goes straight in: it is a change
    /// the reader sees, and one candidate to make it is no choice at all. A
    /// candidate the line already spells goes into the layer instead, with the
    /// line left alone — completing a word to itself is no change and no layer,
    /// which is a `Tab` that reads as a dead key on a name that is already
    /// whole. Several candidates go in as the prefix they share, and the layer
    /// opens so the reader can pick between them. None changes nothing, and says
    /// why in the log: a `Tab` that does nothing at all is indistinguishable
    /// from a `Tab` that was not received, which is what made a missing command
    /// list hard to notice.
    fn complete(&mut self) {
        let Some(target) = self.completion_target() else {
            return;
        };
        self.warn_about_the_builtin_table(&target);
        let candidates = self.completions_for(&target);
        let word = target.word();
        match candidates.as_slice() {
            [] => self.explain_no_candidates(&target),
            [only] => {
                if *only == self.input.word_text(&word) {
                    // The name is already whole, so the layer is what is left to
                    // show: opening it on its one candidate says the word was
                    // understood, where a self-replacement would say nothing.
                    self.completion = Some(Completion {
                        candidates,
                        word,
                        selected: Some(0),
                    });
                } else {
                    self.input.replace(&word, only);
                }
            }
            several => {
                let common = common_prefix(several);
                // Only ever characters the candidates agree on, never fewer than
                // what is already typed: matching is case-insensitive, and a
                // candidate spelled differently from the line (a lower-case
                // `m1` meeting `M115`) can share less with its neighbours than
                // the line already has. Taking that prefix would delete what the
                // reader typed.
                let span = if common.chars().count() >= self.input.word_text(&word).chars().count()
                {
                    self.input.replace(&word, &common)
                } else {
                    word
                };
                self.completion = Some(Completion {
                    candidates,
                    word: span,
                    selected: None,
                });
            }
        }
    }

    /// Say once that the candidates are the built-in table's, not the printer's.
    ///
    /// The printer is asked for its command table whenever g-code mode is
    /// entered and again on every `Tab` until it answers, so by the time a
    /// completion runs the ask has just been made — and a refusal, which is what
    /// a printer still coming up sends, is not an answer. Which table the names
    /// came from is worth one line in the log, and only one: completing a
    /// command the machine does not have looks exactly like completing from a
    /// list it does, and a line per `Tab` would bury the log in the same fact.
    fn warn_about_the_builtin_table(&mut self, target: &Target) {
        if !self.gcode
            || self.gcode_commands_asked
            || self.gcode_fallback_warned
            // A local word is the window's own list, and needs no printer.
            || target.is_local()
        {
            return;
        }
        self.gcode_fallback_warned = true;
        self.write(Entry::notice(
            Notice::Problem,
            "no G-code parameters from the printer; using the built-in table",
        ));
    }

    /// Say why a `Tab` found nothing, when there is a source that should have.
    ///
    /// Only g-code mode has anything to say: its names come from the printer,
    /// whose answer either never arrived or has no name by that prefix. A bare
    /// word in request mode draws from nothing by design (a method name is the
    /// printer's and the window has no list of those), so a notice there would
    /// explain a non-feature.
    ///
    /// The notice goes to the log rather than the input line, so what was typed
    /// is left exactly as it was.
    fn explain_no_candidates(&mut self, target: &Target) {
        let text = match target {
            // A `/` word is answered from the local commands even here, so a
            // miss is the window's own list talking, not the printer's.
            Target::Command { prefix, .. } if target.is_local() => {
                format!("no local command starts with \"{prefix}\"")
            }
            // A bare word in request mode had no list to begin with: there is
            // nothing to explain, and a notice per stray `Tab` would be noise.
            Target::Command { .. } if !self.gcode => return,
            Target::Command { prefix, .. } if self.gcode_commands_asked => {
                if self.gcode_commands.is_empty() {
                    // The printer did answer and its list was empty: there is
                    // nothing to retry, so the notice reports what came back.
                    "the printer answered objects/query but listed no commands".to_string()
                } else {
                    format!(
                        "no G-Code command starts with \"{prefix}\" (the printer reports {} commands)",
                        self.gcode_commands.len()
                    )
                }
            }
            Target::Command { prefix, .. } => format!(
                "no G-Code command starts with \"{prefix}\" (the built-in table has {} commands)",
                gcode_params::BUILTIN.len()
            ),
            Target::Parameter {
                command, prefix, ..
            } => {
                if self.parameter_names(command).is_empty() {
                    // Neither source names a parameter of this command, so the
                    // miss is not about the prefix at all.
                    format!("no parameter names are known for {command}")
                } else {
                    format!("no parameter of {command} starts with \"{prefix}\"")
                }
            }
        };
        self.write(Entry::notice(Notice::Info, text));
    }

    /// Pick the next or previous candidate, and put it in the line.
    ///
    /// The list wraps in both directions, which is what makes holding `Tab`
    /// workable: the reader sees the whole list go past rather than having to
    /// find their way back. The word in the line follows the pick, so what is
    /// left to type is whatever argument comes after it.
    fn cycle_completion(&mut self, step: isize) {
        let Some(completion) = self.completion.as_mut() else {
            return;
        };
        let count = completion.candidates.len();
        if count == 0 {
            return;
        }
        let next = match completion.selected {
            // The first `Tab` on a layer that has nothing picked takes the first
            // candidate; `BackTab` on one takes the last, so that both keys move
            // away from where the layer opened rather than nowhere.
            None if step > 0 => 0,
            None => count - 1,
            Some(index) => (index as isize + step).rem_euclid(count as isize) as usize,
        };
        completion.selected = Some(next);
        let candidate = completion.candidates[next].clone();
        // Each pick replaces the word as the line has it now — the prefix, or
        // the candidate before this one — and the word moves to wherever the new
        // candidate ends, which is where the next pick will replace from.
        let word = self.input.replace(&completion.word, &candidate);
        completion.word = word;
    }

    /// What the word at the caret is a name of, if it is a name of anything.
    ///
    /// The first word of a line names the line: a local command in either mode
    /// (`/help`), or a G-Code command in g-code mode. A later word names an
    /// argument, and the only arguments the window has names for are a G-Code
    /// command's parameters — which is also why a `/`-line's arguments and a
    /// request's are not completed: the first are the session's business and the
    /// second the printer's.
    ///
    /// `None` is the same "nothing here to complete" the word itself can say,
    /// plus one case of its own: a word to the right of the `=` in it. That is a
    /// value, and a value is the printer's; only the names on the left are the
    /// window's.
    fn completion_target(&self) -> Option<Target> {
        let word = self.input.completion_word()?;
        // Nothing but blanks before the word makes it the first one: an
        // indented `/help` is still the command the line is.
        if self.input.buffer[..word.start]
            .iter()
            .all(|character| character.is_whitespace())
        {
            return Some(Target::Command {
                word,
                prefix: self.input.word_text(&word),
            });
        }
        if !self.gcode {
            return None;
        }
        let command = self.input.first_word()?;
        if command.starts_with('/') {
            return None;
        }
        let equals = self.input.buffer[word.start..word.end]
            .iter()
            .position(|character| *character == '=')
            .map(|offset| word.start + offset);
        if equals.is_some_and(|equals| self.input.cursor > equals) {
            return None;
        }
        // The word ends at the `=` when there is one, so that replacing it —
        // which `Tab` does as the reader walks the candidates — leaves the value
        // beside it where it was, and only the name changes.
        let prefix: String = self.input.buffer[word.start..self.input.cursor]
            .iter()
            .collect();
        Some(Target::Parameter {
            command,
            word: Word {
                start: word.start,
                end: equals.unwrap_or(word.end),
            },
            prefix,
        })
    }

    /// The names the word at the caret could be completed to.
    ///
    /// What `complete` works from, so that a test can look at the candidates
    /// without a keypress making a layer of them.
    fn completions_for(&self, target: &Target) -> Vec<String> {
        match target {
            Target::Command { prefix, .. } if target.is_local() => local_command_candidates(prefix),
            Target::Command { prefix, .. } if self.gcode => matching(&self.command_names(), prefix),
            Target::Command { .. } => Vec::new(),
            Target::Parameter {
                command, prefix, ..
            } => matching(&self.parameter_names(command), prefix),
        }
    }

    /// The command names a line completes from: the printer's, once it has
    /// answered, and the built-in table's until then.
    ///
    /// A printer that is up knows better than a table built from the sources —
    /// it knows what its configuration actually registered, and the names it
    /// spells are the ones that run — but one that has not answered leaves `Tab`
    /// with nothing at all, which is worse than a table for a printer of this
    /// same host.
    fn command_names(&self) -> Vec<String> {
        if self.gcode_commands_asked {
            return self.gcode_commands.clone();
        }
        gcode_params::BUILTIN
            .iter()
            .map(|(name, _)| (*name).to_string())
            .collect()
    }

    /// The parameter names of `command`: the printer's if it named any, and the
    /// built-in table's otherwise.
    ///
    /// The printer's answer wins where there is one, for the same reason its
    /// command names do. A command it named none for says nothing at all — a
    /// host too old to report parameters is exactly the case this falls back
    /// for — and the built-in table then has the names the command declared
    /// upstream. Lookups are without case: the table is keyed by the name the
    /// source spells, and a line may be typed in lower case.
    fn parameter_names(&self, command: &str) -> Vec<String> {
        let name = command.to_ascii_uppercase();
        if let Some(named) = self.gcode_parameters.get(&name) {
            if !named.is_empty() {
                return named.clone();
            }
        }
        builtin_parameters(&name)
            .iter()
            .map(|parameter| (*parameter).to_string())
            .collect()
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
    fn test_a_webhooks_push_moves_the_header_on() {
        // This is what the window subscribes to at startup. The header's first
        // value is the handshake's `info` reply, which nothing repeats; a
        // `webhooks` push is the only thing that replaces it later.
        let mut app = App::new();
        app.write(Entry::Push(serde_json::json!({
            "id": null,
            "method": "klippy:status",
            "params": {"eventtime": 12.5, "status": {"webhooks": {
                "state": "shutdown",
                "state_message": "Printer is halted"
            }}}
        })));

        let rows = render(&app, 60, 6);
        assert!(rows[0].contains("shutdown"), "{rows:?}");
        assert!(rows[0].contains("Printer is halted"), "{rows:?}");
    }

    #[test]
    fn test_the_subscribe_reply_snapshot_moves_the_header_on() {
        // The reply to the startup `objects/subscribe` is `webhooks` as it was
        // when the subscription was taken; everything the printer pushes after
        // it is a *change*. A state that settled in between would otherwise
        // never reach the header, which still shows what the handshake's `info`
        // said.
        let mut app = App::new();
        // By the time a subscription is answered the handshake's `info` has
        // been, so the reply is an ordinary entry rather than the greeting.
        app.greeted = true;
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(3),
            method: Some("objects/subscribe".to_string()),
            message: serde_json::json!({
                "id": 3,
                "result": {"eventtime": 12.5, "status": {"webhooks": {
                    "state": "ready",
                    "state_message": "Printer is ready"
                }}}
            }),
        }));

        let rows = render(&app, 60, 6);
        assert!(rows[0].contains("ready"), "{rows:?}");
        assert!(rows[0].contains("Printer is ready"), "{rows:?}");
    }

    #[test]
    fn test_a_failed_subscribe_reply_leaves_the_header_alone() {
        // A refusal carries no `status`, and a printer that told the window the
        // subscription did not take is not news about the printer's state: the
        // header keeps what it has.
        let mut app = App::new();
        app.greeted = true;
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(3),
            method: Some("objects/subscribe".to_string()),
            message: serde_json::json!({
                "id": 3,
                "error": {"error": "CommandError", "message": "Printer is not ready"},
                "result": {"status": {"webhooks": {
                    "state": "ready",
                    "state_message": "Printer is ready"
                }}}
            }),
        }));

        assert_eq!(app.status, Status::Unknown, "nothing was applied");
        let rows = render(&app, 60, 6);
        assert!(!rows[0].contains("ready"), "{rows:?}");
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
    fn test_three_escapes_in_a_row_are_the_emergency_stop() {
        let mut app = App::new();
        let start = tokio::time::Instant::now();
        assert_eq!(app.escape(start), Escape::Wait);
        assert_eq!(app.escape(start + Duration::from_millis(100)), Escape::Wait);
        assert_eq!(
            app.escape(start + Duration::from_millis(200)),
            Escape::Stop,
            "three presses inside the window stop the printer"
        );
        // The gesture is spent: a fourth press starts over rather than stopping
        // the printer a second time.
        assert_eq!(app.escape(start + Duration::from_millis(300)), Escape::Wait);
    }

    #[test]
    fn test_a_lone_escape_does_not_leave_and_stops_counting() {
        let mut app = App::new();
        let start = tokio::time::Instant::now();
        assert_eq!(app.escape(start), Escape::Wait, "Esc is not a quit");
        assert!(
            !app.escape_window_closed(start + Duration::from_millis(100)),
            "still inside the window: the user may be starting the gesture"
        );
        assert!(
            app.escape_window_closed(start + ESTOP_WINDOW),
            "a press the gesture never followed up on stops counting"
        );
        assert!(!app.escape_window_closed(start + ESTOP_WINDOW + Duration::from_millis(1)));
        // A press after the window is a fresh gesture, not a second press.
        assert_eq!(app.escape(start + ESTOP_WINDOW * 2), Escape::Wait);
        assert_eq!(app.escape_streak, 1);
    }

    #[test]
    fn test_escapes_spread_out_are_not_a_stop() {
        let mut app = App::new();
        let start = tokio::time::Instant::now();
        // Each press is a new gesture: the streak never reaches three.
        assert_eq!(app.escape(start), Escape::Wait);
        assert_eq!(app.escape(start + ESTOP_WINDOW), Escape::Wait);
        assert_eq!(app.escape(start + ESTOP_WINDOW * 2), Escape::Wait);
    }

    #[test]
    fn test_the_hint_counts_the_emergency_stop_gesture() {
        let mut app = App::new();
        let start = tokio::time::Instant::now();
        assert_eq!(app.escape_hint(start), None, "nothing pending: no hint");
        assert_eq!(app.escape(start), Escape::Wait);
        let first = app
            .escape_hint(start + Duration::from_millis(10))
            .expect("a press is pending");
        assert!(first.contains("1/3"), "{first}");
        assert_eq!(app.escape(start + Duration::from_millis(50)), Escape::Wait);
        let second = app
            .escape_hint(start + Duration::from_millis(60))
            .expect("two presses are pending");
        assert!(second.contains("2/3"), "{second}");
        assert_eq!(app.escape(start + Duration::from_millis(100)), Escape::Stop);
        assert_eq!(app.escape_hint(start), None, "the stop ends the gesture");
        // A streak the user walked away from stops being advertised.
        assert_eq!(app.escape(start + ESTOP_WINDOW * 2), Escape::Wait);
        assert_eq!(
            app.escape_hint(start + ESTOP_WINDOW * 3),
            None,
            "an expired streak is not shown"
        );
    }

    #[test]
    fn test_the_hint_line_names_the_keys() {
        // Rendered wider than the footer's own budget, so the width check below
        // measures the string instead of the terminal: `render` only collects
        // the cells the terminal has, which would cap every line at the width it
        // was handed and make the check true whatever the footer says.
        let rows = render(&app_with(Vec::new()), 120, 6);
        let footer = rows.last().unwrap();
        assert!(footer.contains("Enter send"), "{footer}");
        assert!(footer.contains("^C quit"), "{footer}");
        // The log's `Home/End` gave up their place here to `Tab complete`; the
        // footer is one line and is kept inside 100 columns.
        assert!(footer.contains("PgUp/PgDn"), "{footer}");
        assert!(footer.contains("Tab complete"), "{footer}");
        assert!(footer.chars().count() <= 100, "{footer}");
    }

    #[test]
    fn test_the_mouse_can_be_handed_back_to_the_terminal() {
        let mut app = app_with(Vec::new());
        assert!(app.mouse, "the window holds the mouse to begin with");

        // A captured mouse is the window's, so the terminal never sees a drag
        // and cannot select. `/mouse` (and `^S`) hands it over; the footer is
        // where that is said, since a log line would land under the frozen pane.
        assert!(app.window_command("/mouse"));
        assert!(!app.mouse);

        let rows = render(&app, 60, 5);
        assert!(rows.last().unwrap().contains("mouse released"), "{rows:?}");

        // Asking again changes nothing: the mouse is already the terminal's.
        assert!(app.window_command("/mouse"));
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
        for character in "/help".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
        let rows = render(&app, 40, 5);
        assert!(rows[3].starts_with("local> /help"), "{rows:?}");
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
        for character in "/help".chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
        let rows = render(&app, 60, 5);
        assert!(rows[3].starts_with("local> /help"), "{rows:?}");
    }

    #[test]
    fn test_gcode_mode_strips_the_script_envelope_to_the_typed_line() {
        // The `gcode/script` request shows as the bare G-Code line, not the
        // `gcode/script` envelope — the pane reads like a G-Code terminal.
        let entry = Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "SET_PIN PIN=fan VALUE=1"}
            }),
        };

        assert_eq!(
            entry_text(
                &entry,
                Render {
                    format: Format::Yaml,
                    gcode: true
                }
            ),
            "> SET_PIN PIN=fan VALUE=1"
        );
        // Request mode keeps the envelope, so the same entry is unchanged.
        assert!(
            entry_text(&entry, Render::DEFAULT).starts_with("> id: 2\n  method: gcode/script"),
            "request mode keeps the envelope"
        );
    }

    #[test]
    fn test_gcode_mode_marks_the_output_push_with_the_received_marker() {
        // Klipper writes `// ` on `respond_info` lines and `!! ` on errors; in
        // g-code mode the window puts its own `<` in front of that line rather
        // than rewriting it, so the text stays the printer's.
        let gcode = Render {
            format: Format::Yaml,
            gcode: true,
        };
        let info = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "// echo: SET_PIN PIN=fan VALUE=1"}
        }));
        let error = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "!! Unknown command"}
        }));
        let plain = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "FIRMWARE_NAME: Klipper"}
        }));
        let multi = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "// A: help\n// B: help"}
        }));
        let empty = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": ""}
        }));

        assert_eq!(
            entry_text(&info, gcode),
            "< // echo: SET_PIN PIN=fan VALUE=1",
            "the `// ` the printer wrote is left alone"
        );
        assert_eq!(entry_text(&error, gcode), "< !! Unknown command");
        assert_eq!(entry_text(&plain, gcode), "< FIRMWARE_NAME: Klipper");
        assert_eq!(
            entry_text(&multi, gcode),
            "< // A: help\n  // B: help",
            "each line stays the printer's; the continuation sits under the marker"
        );
        assert_eq!(
            entry_text(&empty, gcode),
            "<",
            "an empty line gets the marker by itself"
        );
        // Request mode keeps the envelope behind a `<` marker.
        assert!(
            entry_text(&info, Render::DEFAULT).contains("method: gcode:output"),
            "request mode keeps the envelope"
        );
    }

    #[test]
    fn test_gcode_mode_drops_only_successful_script_and_plumbing_replies() {
        // The empty `{}` reply to `gcode/script` carries nothing the output
        // push did not already show, so it is dropped in g-code mode.
        let script_reply = Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(2),
            method: Some("gcode/script".to_string()),
            message: serde_json::json!({"id": 2, "result": {}}),
        });
        // A failed `gcode/script` is kept. Normally the `!! …` output line is
        // the thing to read, but with no dispatcher up there is no output
        // subscription at all, and this reply is the only error there is.
        let script_error = Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(3),
            method: Some("gcode/script".to_string()),
            message: serde_json::json!({
                "id": 3,
                "error": {"error": "WebRequestError", "message": "Printer is halted"}
            }),
        });
        // The subscription's request and empty reply are plumbing a notice
        // already announced, so they are dropped as well.
        let sub_sent = Entry::Sent {
            id: Some(1),
            method: "gcode/subscribe_output".to_string(),
            message: serde_json::json!({
                "id": 1,
                "method": "gcode/subscribe_output",
                "params": {"response_template": {"method": "gcode:output"}}
            }),
        };
        let sub_reply = Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("gcode/subscribe_output".to_string()),
            message: serde_json::json!({"id": 1, "result": {}}),
        });
        // Its failure is not dropped: "the printer has no dispatcher yet" is
        // exactly the error no output line can carry.
        let sub_error = Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("gcode/subscribe_output".to_string()),
            message: serde_json::json!({
                "id": 1,
                "error": {"error": "CommandError", "message": "Printer is not ready"}
            }),
        });

        assert!(gcode_suppress(&script_reply), "the script reply is dropped");
        assert!(
            !gcode_suppress(&script_error),
            "a failed script reply is kept"
        );
        assert!(
            gcode_suppress(&sub_sent),
            "the subscription request is dropped"
        );
        assert!(
            gcode_suppress(&sub_reply),
            "the subscription reply is dropped"
        );
        assert!(
            !gcode_suppress(&sub_error),
            "a failed subscription reply is kept"
        );
    }

    #[test]
    fn test_gcode_mode_keeps_a_failed_subscription_in_the_log() {
        // End to end, the dispatcher-less case: the subscription fails, so the
        // `gcode:output` push never comes and the only trace of the problem is
        // the error reply — which must therefore reach the pane.
        let mut app = app_with(Vec::new());
        app.gcode = true;
        app.render.gcode = true;

        app.write(Entry::Sent {
            id: Some(1),
            method: "gcode/subscribe_output".to_string(),
            message: serde_json::json!({
                "id": 1,
                "method": "gcode/subscribe_output",
                "params": {"response_template": {"method": "gcode:output"}}
            }),
        });
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("gcode/subscribe_output".to_string()),
            message: serde_json::json!({
                "id": 1,
                "error": {"error": "CommandError", "message": "Printer is not ready"}
            }),
        }));

        let joined = log_text(&app, 60, 8).join("\n");
        assert!(
            joined.contains("Printer is not ready"),
            "the error is visible: {joined}"
        );
        assert!(
            !joined.contains("method: gcode/subscribe_output"),
            "but not as an envelope: {joined}"
        );
    }

    #[test]
    fn test_request_mode_shows_the_gcode_plumbing_the_window_sends() {
        // The stripping is g-code mode's alone: entering the mode sends
        // `gcode/subscribe_output` itself, so in request mode the same request
        // and reply are ordinary entries behind their envelopes.
        let mut app = app_with(Vec::new());
        assert!(!app.gcode, "the window starts in request mode");

        app.write(Entry::Sent {
            id: Some(1),
            method: "gcode/subscribe_output".to_string(),
            message: serde_json::json!({
                "id": 1,
                "method": "gcode/subscribe_output",
                "params": {"response_template": {"method": "gcode:output"}}
            }),
        });
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("gcode/subscribe_output".to_string()),
            message: serde_json::json!({"id": 1, "result": {}}),
        }));

        assert_eq!(app.entries.len(), 2, "nothing is dropped in request mode");
        let joined = log_text(&app, 60, 12).join("\n");
        assert!(
            joined.contains("method: gcode/subscribe_output"),
            "the envelope stays: {joined}"
        );
    }

    #[test]
    fn test_changing_the_render_options_remeasures_the_log() {
        // The measured heights are keyed on the render options, not only on the
        // pane width: the same entry is shorter once its envelope is stripped,
        // so flipping the option has to throw the cached heights away. The
        // width is set first so the render options are the *only* key that
        // differs between the two measurements.
        let mut app = app_with(Vec::new());
        app.width.set(60);
        app.write(Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "M115"}
            }),
        });

        assert_eq!(app.total_lines(60), 4, "the envelope, at 60 columns");

        // Flipping the option changes no entry and no width: only the render
        // key can force the log to be measured again.
        app.render.gcode = true;
        assert_eq!(app.total_lines(60), 1, "stripped to the typed line");
    }

    #[test]
    fn test_gcode_mode_keeps_the_script_request_and_output_push() {
        // The `gcode/script` request and the `gcode:output` push are the
        // exchange itself, so they are kept — the renderer rewrites them.
        let script_sent = Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "M115"}
            }),
        };
        let output_push = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "FIRMWARE_NAME: Klipper"}
        }));

        assert!(!gcode_suppress(&script_sent), "the script request is kept");
        assert!(!gcode_suppress(&output_push), "the output push is kept");
    }

    #[test]
    fn test_gcode_mode_does_not_touch_other_channels() {
        // A `klippy:status` push and an `objects/query` reply are not g-code
        // traffic, so g-code mode leaves them alone: still shown, still behind
        // their envelopes.
        let status = Entry::Push(serde_json::json!({
            "method": "klippy:status",
            "params": {"status": {"webhooks": {"state": "ready"}}}
        }));
        let query = Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(4),
            method: Some("objects/query".to_string()),
            message: serde_json::json!({"id": 4, "result": {}}),
        });

        assert!(
            !gcode_suppress(&status),
            "a status push is never suppressed"
        );
        assert!(!gcode_suppress(&query), "a query reply is never suppressed");
        assert!(
            entry_text(
                &status,
                Render {
                    format: Format::Yaml,
                    gcode: true
                }
            )
            .starts_with("< method: klippy:status"),
            "non-g-code traffic keeps its envelope in g-code mode"
        );
    }

    #[test]
    fn test_gcode_mode_renders_the_exchange_without_the_envelope() {
        // End to end: a script request, its output push, and the plumbing the
        // window drops, all written through `Output` in g-code mode. The pane
        // shows the typed line and the printer's reply, and nothing else from
        // the `gcode/*` plumbing.
        let mut app = app_with(Vec::new());
        app.gcode = true;
        app.render.gcode = true;

        app.write(Entry::Sent {
            id: Some(1),
            method: "gcode/subscribe_output".to_string(),
            message: serde_json::json!({
                "id": 1,
                "method": "gcode/subscribe_output",
                "params": {"response_template": {"method": "gcode:output"}}
            }),
        });
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(1),
            method: Some("gcode/subscribe_output".to_string()),
            message: serde_json::json!({"id": 1, "result": {}}),
        }));
        app.write(Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "M115"}
            }),
        });
        app.write(Entry::Reply(crate::connection::Reply {
            id: serde_json::json!(2),
            method: Some("gcode/script".to_string()),
            message: serde_json::json!({"id": 2, "result": {}}),
        }));
        app.write(Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "FIRMWARE_NAME: Klipper"}
        })));

        let lines = log_text(&app, 60, 8);
        let joined = lines.join("\n");
        assert!(joined.contains("> M115"), "the typed line: {joined}");
        assert!(
            joined.contains("< FIRMWARE_NAME: Klipper"),
            "the printer's line, marked as received: {joined}"
        );
        assert!(!joined.contains("gcode/script"), "no envelope: {joined}");
        assert!(
            !joined.contains("gcode:output"),
            "no push envelope: {joined}"
        );
        assert!(
            !joined.contains("subscribe_output"),
            "no plumbing: {joined}"
        );
    }

    #[test]
    fn test_request_mode_after_gcode_shows_the_envelope_again() {
        // Toggling back to request mode re-wraps the log, so the same `gcode/
        // script` exchange shows its envelope again.
        let mut app = app_with(Vec::new());
        app.gcode = true;
        app.render.gcode = true;
        app.write(Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "M115"}
            }),
        });

        // G-code mode: the bare typed line.
        let gcode_lines = log_text(&app, 60, 6);
        assert!(gcode_lines.join("\n").contains("> M115"));

        // Back to request mode: the envelope returns.
        app.toggle_gcode();
        let request_lines = log_text(&app, 60, 6);
        let joined = request_lines.join("\n");
        assert!(
            joined.contains("method: gcode/script"),
            "envelope is back: {joined}"
        );
    }

    #[test]
    fn test_gcode_mode_colours_the_directions() {
        // The window's markers are the only scaffolding in g-code mode, so the
        // colour carries the direction: typed lines cyan, the printer's output
        // white, and a `!! ` line red like any other failure.
        let script = Entry::Sent {
            id: Some(2),
            method: "gcode/script".to_string(),
            message: serde_json::json!({
                "id": 2,
                "method": "gcode/script",
                "params": {"script": "M115"}
            }),
        };
        let info = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "// Klipper version: v0.12.0"}
        }));
        let error = Entry::Push(serde_json::json!({
            "id": null,
            "method": "gcode:output",
            "params": {"response": "!! Unknown command"}
        }));
        let render = Render {
            format: Format::Yaml,
            gcode: true,
        };

        // The indices are picked so the plain alternation would hand out the
        // *other* colour: without the direction branch every assertion below
        // fails, which is the point of the test.
        assert_eq!(entry_style(&script, 1, render).fg, Some(Color::Cyan));
        assert_eq!(entry_style(&info, 0, render).fg, Some(Color::White));
        assert_eq!(entry_style(&error, 1, render).fg, Some(Color::Red));
    }

    #[test]
    fn test_gcode_mode_entries_still_count_toward_the_alternation() {
        // g-code lines are drawn in their own colours, but they are messages
        // like any other: the two status pushes around one still share a colour
        // because the g-code line in between took the second slot. Drop it from
        // the count and the second push would take the other colour instead.
        let mut app = app_with(vec![
            Entry::Push(serde_json::json!({"method": "klippy:status"})),
            Entry::Push(serde_json::json!({
                "id": null,
                "method": "gcode:output",
                "params": {"response": "// ok"}
            })),
            Entry::Push(serde_json::json!({"method": "klippy:status"})),
        ]);
        app.gcode = true;
        app.render.gcode = true;

        let total = app.total_lines(40);
        let lines = visible_lines(&app, 40, 10, total).lines;
        let colours: Vec<Option<Color>> = lines
            .iter()
            .filter(|line| line.spans[0].content.starts_with('<'))
            .map(|line| line.spans[0].style.fg)
            .collect();

        assert_eq!(
            colours,
            vec![
                Some(MESSAGE_COLORS[0]),
                Some(Color::White),
                Some(MESSAGE_COLORS[0])
            ],
            "{lines:?}"
        );
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
        assert_eq!(
            entry_style(&sent, 0, Render::DEFAULT).fg,
            Some(MESSAGE_COLORS[0])
        );
        assert_eq!(
            entry_style(&Entry::Push(serde_json::json!({})), 1, Render::DEFAULT).fg,
            Some(MESSAGE_COLORS[1])
        );
        assert_eq!(
            entry_style(&Entry::Reply(ok), 2, Render::DEFAULT).fg,
            Some(MESSAGE_COLORS[0])
        );

        // A failed request is red wherever it falls in the alternation.
        assert_eq!(
            entry_style(&Entry::Reply(failed.clone()), 0, Render::DEFAULT).fg,
            Some(Color::Red)
        );
        assert_eq!(
            entry_style(&Entry::Reply(failed), 1, Render::DEFAULT).fg,
            Some(Color::Red)
        );

        // The window's own lines keep their colours.
        assert_eq!(
            entry_style(&Entry::notice(Notice::Problem, "x"), 0, Render::DEFAULT).fg,
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
        let text = entry_text(&Entry::Reply(query_reply()), Render::DEFAULT);

        assert!(text.starts_with("< id: 2\n  result:\n"), "{text}");
        assert!(text.contains("eventtime: 1.5"), "{text}");
        assert!(text.contains("mcu_version: abc"), "{text}");
        // Not the compact JSON the wire carries.
        assert!(!text.contains("{\"eventtime\""), "{text}");
    }

    #[test]
    fn test_json_keeps_the_marker_and_the_wire_form() {
        let text = entry_text(
            &Entry::Reply(query_reply()),
            Render {
                format: Format::Json,
                gcode: false,
            },
        );

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
            Render::DEFAULT,
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
            entry_text(&entry, Render::DEFAULT),
            "> id: 2\n  method: objects/query"
        );
        assert_eq!(
            entry_text(
                &entry,
                Render {
                    format: Format::Json,
                    gcode: false
                }
            ),
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
                entry_text(
                    &Entry::Reply(reply.clone()),
                    Render {
                        format,
                        gcode: false
                    }
                ),
                "! 3 (gcode/script) Printer is halted"
            );
        }
    }

    #[test]
    fn test_the_format_command_switches_and_says_so() {
        let mut app = app_with(Vec::new());
        assert_eq!(app.render.format, Format::Yaml, "YAML is the default");

        assert!(app.window_command("/json"));
        assert_eq!(app.render.format, Format::Json);
        assert!(app.window_command("/yaml"));
        assert_eq!(app.render.format, Format::Yaml);
        assert!(
            !app.window_command("/subscribe"),
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
                Render::DEFAULT,
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
        let style = entry_style(&entry, 0, Render::DEFAULT);

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
            entry_style(&message, 0, Render::DEFAULT),
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

    // -----------------------------------------------------------------------
    // Completion
    // -----------------------------------------------------------------------

    /// Type `line` into the input line, a press at a time.
    fn type_line(app: &mut App, line: &str) {
        for character in line.chars() {
            app.input.edit(KeyCode::Char(character), false);
        }
    }

    /// The candidates `Tab` would find at the caret, without the keypress and
    /// so without the layer or the notice one might make.
    fn candidates_at(app: &App) -> Vec<String> {
        match app.completion_target() {
            Some(target) => app.completions_for(&target),
            None => Vec::new(),
        }
    }

    /// An app in g-code mode with the printer's command table already cached,
    /// which is what a bare g-code word completes from.
    fn app_with_gcode_commands(names: &[&str]) -> App {
        let mut app = app_with(Vec::new());
        app.gcode = true;
        app.gcode_commands = names.iter().map(|name| (*name).to_string()).collect();
        // Cached means answered: the flag is what says the printer had its say,
        // including the say that there are no commands.
        app.gcode_commands_asked = true;
        app
    }

    /// An app in g-code mode whose printer has not answered: nothing is cached,
    /// so completion draws on the built-in table.
    fn app_with_no_printer_commands() -> App {
        let mut app = app_with(Vec::new());
        app.gcode = true;
        app
    }

    /// The command names the real printer reports, in the sorted order the
    /// session hands them over: the described ones and the built-ins with no
    /// description (`M115`, `M110`, `ECHO`) alike.
    const PRINTER_COMMANDS: [&str; 13] = [
        "ECHO",
        "FIRMWARE_RESTART",
        "HELP",
        "M110",
        "M112",
        "M115",
        "RESTART",
        "SET_GCODE_VARIABLE",
        "SET_PIN",
        "STATUS",
        "STEPPER_MOVE",
        "STEPPER_RELEASE",
        "_STEPPER_SET_PHASE",
    ];

    #[test]
    fn test_the_word_at_the_caret_is_the_one_under_it() {
        // Indented, the word is still the first word.
        let mut input = Input::default();
        for character in "  /hel".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        let word = input.completion_word().expect("an indented word");
        assert_eq!(word, Word { start: 2, end: 6 });
        assert_eq!(input.word_text(&word), "/hel");
        assert_eq!(input.first_word().as_deref(), Some("/hel"));

        // A word after the first one is a word too — what the word *is* is the
        // caller's question, and an argument of a local line is still none of
        // the window's business.
        input.edit(KeyCode::Char(' '), false);
        input.edit(KeyCode::Char('x'), false);
        let word = input.completion_word().expect("the argument's word");
        assert_eq!(word, Word { start: 7, end: 8 });
        assert_eq!(input.first_word().as_deref(), Some("/hel"));

        // A blank under the caret is the gap between two words, not a word.
        let mut input = Input::default();
        for character in "SET_PIN  PIN".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        input.cursor = 8;
        assert!(
            input.completion_word().is_none(),
            "the caret is between the two blanks: {}",
            input.text()
        );

        // A line of blanks has no word in it either.
        let mut input = Input::default();
        for character in "   ".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        assert!(input.completion_word().is_none());
        assert!(input.first_word().is_none());

        // An empty line, though, is a word of no characters: `Tab` there lists
        // every candidate rather than nothing.
        let input = Input::default();
        assert_eq!(input.completion_word(), Some(Word { start: 0, end: 0 }));

        // So is the end of a line that has something on it: `SET_PIN PIN=fan `
        // has a word to start after the blank.
        let mut input = Input::default();
        for character in "SET_PIN PIN=fan ".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        assert_eq!(input.completion_word(), Some(Word { start: 16, end: 16 }));

        // The caret inside the word takes the whole of it.
        let mut input = Input::default();
        for character in "/help".chars() {
            input.edit(KeyCode::Char(character), false);
        }
        input.edit(KeyCode::Home, false);
        input.edit(KeyCode::Right, false);
        assert_eq!(input.completion_word(), Some(Word { start: 0, end: 5 }));
    }

    #[test]
    fn test_tab_completes_the_only_candidate() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/hel");
        assert!(
            app.completion_key(KeyCode::Tab),
            "Tab is the completion key"
        );
        assert_eq!(app.input.text(), "/help");
        assert!(app.completion.is_none(), "one candidate needs no layer");
        assert_eq!(app.input.cursor, 5, "the caret is after the word");

        // `/h` is an alias of `/help`, and `Tab` finds the command by prefix
        // like any other: what goes in the line is the canonical name.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/h");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "/help");
    }

    #[test]
    fn test_tab_narrows_to_the_shared_prefix_and_opens_the_layer() {
        let mut app = app_with_gcode_commands(&["M104", "M115", "M140"]);
        type_line(&mut app, "m1");
        assert!(app.completion_key(KeyCode::Tab));
        // The candidates all start `M1`, so the line gains what they agree on —
        // spelled the way the printer spells it, since that is what runs.
        assert_eq!(app.input.text(), "M1");
        let completion = app.completion.as_ref().expect("the layer is open");
        assert_eq!(completion.candidates, ["M104", "M115", "M140"]);
        assert_eq!(completion.selected, None, "nothing is picked yet");
        assert_eq!(completion.word, Word { start: 0, end: 2 });
    }

    #[test]
    fn test_a_shared_prefix_never_shortens_what_was_typed() {
        // Matching is case-insensitive, so two candidates can agree on less than
        // the line already has: `m1` reaches `M115` and `m115`, which share no
        // first character at all. Filling in that prefix would delete the word
        // the reader typed, so the line keeps it and only the layer opens.
        let mut app = app_with_gcode_commands(&["M115", "m115"]);
        type_line(&mut app, "m1");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "m1", "the typed word survives");
        let completion = app.completion.as_ref().expect("the layer is open");
        assert_eq!(completion.candidates, ["M115", "m115"]);
        assert_eq!(completion.word, Word { start: 0, end: 2 });
    }

    #[test]
    fn test_tab_on_a_word_with_no_candidates_does_nothing() {
        // A bare word in request mode has no source at all — not even a notice:
        // nothing could have answered, so there is nothing to explain.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "objects/quer");
        assert!(app.completion_key(KeyCode::Tab), "the key is still Tab's");
        assert_eq!(app.input.text(), "objects/quer");
        assert!(app.completion.is_none());
        assert!(app.entries.is_empty(), "request mode stays silent");

        // A `/` word does have a source (the local commands), so a miss is
        // explained — in either mode — and the line is still left alone.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/zz");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "/zz");
        assert!(app.completion.is_none());
        assert_eq!(app.entries.len(), 1, "the miss is explained");
        assert!(
            app.entries
                .last()
                .unwrap()
                .text()
                .contains("no local command"),
            "the notice names the local list"
        );
    }

    #[test]
    fn test_tab_on_an_empty_gcode_line_lists_every_command() {
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        assert!(app.input.text().is_empty());

        assert!(app.completion_key(KeyCode::Tab));

        let completion = app.completion.as_ref().expect("the whole list opens");
        assert_eq!(completion.candidates.len(), 13);
        assert_eq!(completion.selected, None, "nothing is picked yet");
        assert_eq!(completion.word, Word { start: 0, end: 0 });
        assert!(
            completion.candidates.contains(&"M115".to_string()),
            "a built-in with no help is in it too: {:?}",
            completion.candidates
        );
        assert!(app.input.text().is_empty(), "an empty line stays empty");
    }

    #[test]
    fn test_an_unanswered_printer_completes_from_the_built_in_table_once() {
        // The printer never answered — it refused an ask, or was never asked —
        // so `Tab` completes from the built-in table, which knows this host's
        // commands, and says once that that is what it is doing. Which table the
        // names came from is not visible in a candidate.
        let mut app = app_with_no_printer_commands();

        assert!(app.completion_key(KeyCode::Tab));

        let completion = app
            .completion
            .as_ref()
            .expect("the built-in table has commands to offer");
        assert_eq!(completion.candidates.len(), gcode_params::BUILTIN.len());
        assert!(
            completion.candidates.contains(&"SET_PIN".to_string()),
            "the table is the host's own commands"
        );
        assert!(
            !completion.candidates.contains(&"M115".to_string()),
            "and it is not the printer's list: those names are not in it"
        );
        assert_eq!(app.entries.len(), 1, "one notice, and only about the table");
        let text = app.entries[0].text();
        assert!(text.contains("using the built-in table"), "{text}");
        assert!(matches!(
            app.entries[0],
            Entry::Notice {
                kind: Notice::Problem,
                ..
            }
        ));
        assert!(app.input.text().is_empty(), "the notice is not the line");
    }

    #[test]
    fn test_the_built_in_table_is_announced_once_and_not_per_tab() {
        // The same fact holds for every `Tab` until the printer answers, so it
        // is worth saying once and would be noise said again: the log is for
        // what happened, not for what is still the case.
        let mut app = app_with_no_printer_commands();

        for _ in 0..5 {
            // Each press is a fresh `Tab`: the layer is closed the way any other
            // key closes it, so the next one completes again.
            app.completion = None;
            assert!(app.completion_key(KeyCode::Tab));
        }

        let warnings = app
            .entries
            .iter()
            .filter(|entry| {
                matches!(
                    entry,
                    Entry::Notice {
                        kind: Notice::Problem,
                        ..
                    }
                )
            })
            .count();
        assert_eq!(warnings, 1, "one per session: {:?}", app.entries);
    }

    #[test]
    fn test_a_completion_from_the_printers_table_says_nothing() {
        // The names are the machine's own, so there is nothing to explain.
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_some(), "the list is what `Tab` opens");
        assert!(
            app.entries.is_empty(),
            "and it is not news: {:?}",
            app.entries
        );
    }

    #[test]
    fn test_tab_on_an_empty_gcode_line_from_an_answered_printer_says_it_has_none() {
        // The list was asked for and the answer was empty: retrying would be a
        // second round trip for the same nothing, so the notice says what came
        // back instead of offering a retry.
        let mut app = app_with_gcode_commands(&[]);

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_none(), "there is nothing to open");
        assert_eq!(app.entries.len(), 1, "one notice");
        let text = app.entries[0].text();
        assert!(text.contains("listed no commands"), "{text}");
        assert!(
            !text.contains("Tab asks again"),
            "no retry to offer: {text}"
        );
        assert!(app.input.text().is_empty(), "the notice is not the line");
    }

    #[test]
    fn test_a_local_word_in_gcode_mode_is_explained_as_a_local_word() {
        // `/` is answered from the local commands whatever the mode, so a miss
        // there must not be blamed on the printer's command list.
        let mut app = app_with_gcode_commands(&["M115"]);
        type_line(&mut app, "/zz");

        assert!(app.completion_key(KeyCode::Tab));

        let text = app.entries.last().expect("a notice").text();
        assert!(text.contains("no local command starts with"), "{text}");
        assert!(!text.contains("G-Code command starts with"), "{text}");
        assert_eq!(app.input.text(), "/zz", "the line is left alone");
    }

    #[test]
    fn test_tab_with_no_matching_gcode_command_says_what_the_printer_reports() {
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        type_line(&mut app, "zz");

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_none(), "there is nothing to open");
        assert_eq!(app.entries.len(), 1, "one notice");
        let text = app.entries[0].text();
        assert!(text.contains("\"zz\""), "the prefix is named: {text}");
        assert!(text.contains("13 commands"), "and the list's size: {text}");
        assert_eq!(app.input.text(), "zz", "the notice is not the line");
    }

    #[test]
    fn test_tab_on_a_line_of_blanks_still_does_nothing() {
        // Blanks are not an empty line: the reader has started a word, so `Tab`
        // there is not a request for the whole list.
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        type_line(&mut app, "   ");

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_none(), "blanks are not a word");
        assert!(app.entries.is_empty(), "and there is nothing to say");
        assert_eq!(app.input.text(), "   ");
    }

    #[test]
    fn test_tab_leaves_the_arguments_alone() {
        // A local line's arguments are the session's business: the window can
        // name a local command, not what one takes.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/subscribe tool");
        assert_eq!(app.input.first_word().as_deref(), Some("/subscribe"));

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "/subscribe tool");
        assert!(app.completion.is_none());
        assert!(app.entries.is_empty(), "and nothing to explain");

        // A request's arguments are the printer's, and the window has no list
        // of those either.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "objects/query tool");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "objects/query tool");
        assert!(app.completion.is_none());
        assert!(app.entries.is_empty());
    }

    #[test]
    fn test_the_word_after_a_command_completes_from_its_parameters() {
        // The names after the command are the command's parameter names, and
        // the window knows those: the built-in table has them, and this printer
        // has not said otherwise.
        let mut app = app_with_no_printer_commands();
        type_line(&mut app, "SET_PIN PI");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "SET_PIN PIN");
        assert_eq!(app.input.cursor, 11, "the caret is after the name");
    }

    #[test]
    fn test_a_parameter_word_is_completed_left_of_its_equals() {
        // What is typed before the `=` is a name, and completing it replaces the
        // name alone: the value beside it belongs to the printer and stays.
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        type_line(&mut app, "SET_PIN PIN=fan VA");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "SET_PIN PIN=fan VALUE");
        assert_eq!(app.input.cursor, 21, "the caret is after the name");
    }

    #[test]
    fn test_a_value_after_the_equals_is_not_completed() {
        // The caret past the `=` is in a value, and the window has no list of
        // values: there is nothing to offer and nothing to explain.
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        type_line(&mut app, "SET_PIN PIN=fan");

        assert!(app.completion_key(KeyCode::Tab), "the key is still Tab's");

        assert_eq!(app.input.text(), "SET_PIN PIN=fan");
        assert!(app.completion.is_none());
        assert!(app.entries.is_empty(), "and nothing to say");
    }

    #[test]
    fn test_a_blank_after_a_command_lists_its_parameters() {
        // The word at the end of the line is empty, which is the same case the
        // empty line is: everything that could come next.
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        type_line(&mut app, "SET_PIN PIN=fan ");

        assert!(app.completion_key(KeyCode::Tab));

        let completion = app.completion.as_ref().expect("the layer opens");
        assert_eq!(completion.candidates, ["PIN", "VALUE", "CYCLE_TIME"]);
        assert_eq!(completion.word, Word { start: 16, end: 16 });
        assert_eq!(completion.selected, None, "nothing is picked yet");
        assert_eq!(app.input.text(), "SET_PIN PIN=fan ", "the line is entire");
    }

    #[test]
    fn test_the_printers_parameters_win_over_the_built_in_table() {
        // The printer knows what its configuration registered, so its answer is
        // the one that runs — including when it disagrees with the table.
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        app.gcode_parameters.insert(
            "SET_PIN".to_string(),
            vec!["CHANNEL".to_string(), "VALUE".to_string()],
        );
        type_line(&mut app, "SET_PIN ");

        assert!(app.completion_key(KeyCode::Tab));

        let completion = app.completion.as_ref().expect("the layer opens");
        assert_eq!(completion.candidates, ["CHANNEL", "VALUE"]);
        assert!(
            !completion.candidates.contains(&"PIN".to_string()),
            "the table's `PIN` is not in the printer's list: {:?}",
            completion.candidates
        );
    }

    #[test]
    fn test_a_command_the_printer_named_no_parameters_for_uses_the_table() {
        // A key the answer left out says the same as an empty list — the printer
        // named nothing — and the table has what the command declared upstream.
        for named in [None, Some(Vec::new())] {
            let mut app = app_with_gcode_commands(&["SET_PIN"]);
            if let Some(named) = named {
                app.gcode_parameters.insert("SET_PIN".to_string(), named);
            }
            type_line(&mut app, "SET_PIN va");

            assert!(app.completion_key(KeyCode::Tab));

            assert_eq!(
                app.input.text(),
                "SET_PIN VALUE",
                "spelled as the table has it"
            );
        }
    }

    #[test]
    fn test_a_command_with_no_known_parameters_says_so() {
        // `ABORT` declares none, and this printer named none: there is nothing
        // to complete the word with, and the notice says which side is empty
        // rather than blaming the prefix.
        let mut app = app_with_gcode_commands(&["ABORT"]);
        type_line(&mut app, "ABORT X");

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_none());
        assert_eq!(app.entries.len(), 1, "one notice");
        let text = app.entries[0].text();
        assert!(
            text.contains("no parameter names are known for ABORT"),
            "{text}"
        );
        assert_eq!(app.input.text(), "ABORT X");
    }

    #[test]
    fn test_a_parameter_prefix_that_matches_nothing_names_the_command() {
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        type_line(&mut app, "SET_PIN PIN=fan ZZ");

        assert!(app.completion_key(KeyCode::Tab));

        assert!(app.completion.is_none(), "there is nothing to open");
        assert_eq!(app.entries.len(), 1, "one notice");
        let text = app.entries[0].text();
        assert!(
            text.contains("no parameter of SET_PIN starts with \"ZZ\""),
            "{text}"
        );
        assert_eq!(app.input.text(), "SET_PIN PIN=fan ZZ");
    }

    #[test]
    fn test_walking_a_parameters_candidates_leaves_the_value_alone() {
        // Every pick replaces the name the layer is completing and nothing else:
        // the `=fan` beside it is the reader's and stays where it is.
        let mut app = app_with_gcode_commands(&["SET_PIN"]);
        app.gcode_parameters.insert(
            "SET_PIN".to_string(),
            vec!["CHANNEL".to_string(), "CHECK".to_string()],
        );
        type_line(&mut app, "SET_PIN PIN=fan C");

        assert!(app.completion_key(KeyCode::Tab), "the layer opens");
        assert!(app.completion_key(KeyCode::Tab), "on the first candidate");

        assert_eq!(app.input.text(), "SET_PIN PIN=fan CHANNEL");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "SET_PIN PIN=fan CHECK");
    }

    #[test]
    fn test_the_first_word_is_a_command_name_and_not_a_parameter() {
        // The regression guard for the word parameters hang off: `M1` names a
        // command, whatever the command before it in the line might take.
        let mut app = app_with_gcode_commands(&["M104", "SET_PIN"]);
        type_line(&mut app, "M1");

        assert_eq!(candidates_at(&app), ["M104"]);

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "M104");
    }

    #[test]
    fn test_a_completion_replaces_the_whole_word_from_wherever_the_caret_is() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/he");
        app.input.edit(KeyCode::Left, false);
        app.input.edit(KeyCode::Left, false);
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(
            app.input.text(),
            "/help",
            "the whole word, not just the head"
        );
        assert_eq!(app.input.cursor, 5, "and the caret ends up after it");
    }

    #[test]
    fn test_an_indented_local_line_still_completes() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "  /gco");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(
            app.input.text(),
            "  /gcode",
            "the indentation is not the word"
        );
        assert_eq!(app.input.cursor, 8);
    }

    #[test]
    fn test_a_gcode_completion_ignores_case_and_inserts_the_printers_spelling() {
        let mut app = app_with_gcode_commands(&["M115", "M104"]);
        type_line(&mut app, "m115");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "M115");
        assert!(app.completion.is_none(), "one candidate, so no layer");
    }

    #[test]
    fn test_tab_on_a_whole_gcode_name_opens_the_layer_on_it() {
        // The word is already complete, so there is nothing to put in the line:
        // a self-replacement is no change at all, which makes `Tab` look like a
        // key the window did not receive. The layer is the answer instead.
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        type_line(&mut app, "HELP");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "HELP", "the line is not touched");
        let completion = app.completion.as_ref().expect("the layer is open");
        assert_eq!(completion.candidates, ["HELP"], "the one name it matched");
        assert_eq!(completion.selected, Some(0), "and it is the picked one");
    }

    #[test]
    fn test_tab_on_a_lower_case_name_still_takes_the_printers_spelling() {
        // The candidate differs from the line, so it goes in as before: this is
        // the case the exact-match rule must not swallow.
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        type_line(&mut app, "help");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "HELP", "the printer's spelling");
        assert!(app.completion.is_none(), "a change needs no layer");
    }

    #[test]
    fn test_tab_twice_on_a_whole_name_holds_its_place() {
        let mut app = app_with_gcode_commands(&PRINTER_COMMANDS);
        type_line(&mut app, "HELP");
        app.completion_key(KeyCode::Tab);

        assert!(app.completion_key(KeyCode::Tab), "the layer's key again");

        assert_eq!(app.input.text(), "HELP", "still the same word");
        let completion = app.completion.as_ref().expect("the layer stays open");
        assert_eq!(completion.selected, Some(0), "on the only candidate");
    }

    #[test]
    fn test_tab_on_a_whole_local_command_opens_the_layer_on_it() {
        // The window's own commands are whole words too: `/gcode` is the only
        // name it matches, and it is already spelled the way the window spells
        // it.
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/gcode");

        assert!(app.completion_key(KeyCode::Tab));

        assert_eq!(app.input.text(), "/gcode");
        let completion = app.completion.as_ref().expect("the layer is open");
        assert_eq!(completion.candidates, ["/gcode"]);
        assert_eq!(completion.selected, Some(0));
    }

    #[test]
    fn test_the_layer_cycles_through_the_candidates_and_wraps() {
        let mut app = app_with_gcode_commands(&["M104", "M115", "M140"]);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);

        assert!(
            app.completion_key(KeyCode::Tab),
            "the first Tab picks a candidate"
        );
        assert_eq!(app.input.text(), "M104");
        assert_eq!(app.input.cursor, 4, "the caret stays at the word's end");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "M115");
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "M140");
        // Off the end is the beginning again...
        assert!(app.completion_key(KeyCode::Tab));
        assert_eq!(app.input.text(), "M104");
        // ...and the other key goes round the same list the other way.
        assert!(app.completion_key(KeyCode::BackTab));
        assert_eq!(app.input.text(), "M140");
        assert_eq!(app.completion.as_ref().unwrap().selected, Some(2));
    }

    #[test]
    fn test_backtab_with_no_layer_is_not_consumed() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/hel");
        assert!(
            !app.completion_key(KeyCode::BackTab),
            "`Tab` opens the layer"
        );
        assert!(app.completion.is_none());
        assert_eq!(app.input.text(), "/hel");
    }

    #[test]
    fn test_backspace_closes_the_layer_and_still_deletes() {
        let mut app = app_with_gcode_commands(&["M104", "M115"]);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);

        assert!(
            !app.completion_key(KeyCode::Backspace),
            "the layer lets it go"
        );
        assert!(app.completion.is_none(), "and closes on the way");
        app.input.edit(KeyCode::Backspace, false);
        assert_eq!(app.input.text(), "M", "the key did its own job");
    }

    #[test]
    fn test_the_layer_does_not_swallow_the_keys_that_are_not_its_own() {
        // Every one of these has a job of its own in `handle_key`, which only
        // runs if `completion_key` says the key is not the layer's: an open
        // layer must never be a mode the keyboard is stuck in.
        for code in [
            KeyCode::Esc,
            KeyCode::Enter,
            KeyCode::Backspace,
            KeyCode::Delete,
            KeyCode::Up,
            KeyCode::Down,
            KeyCode::Char('c'),
            KeyCode::Char('d'),
            KeyCode::Char('g'),
            KeyCode::Char('l'),
            KeyCode::Char('u'),
        ] {
            let mut app = app_with_gcode_commands(&["M104", "M115"]);
            type_line(&mut app, "m1");
            app.completion_key(KeyCode::Tab);
            assert!(app.completion.is_some(), "the layer is open for {code:?}");

            assert!(!app.completion_key(code), "{code:?} is its own key");
            assert!(app.completion.is_none(), "{code:?} closed the layer");
            assert_eq!(app.input.text(), "M1", "the layer left the line alone");
        }
    }

    #[test]
    fn test_a_local_line_completes_from_the_session_and_the_window_once_each() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "/");
        let candidates = candidates_at(&app);

        assert!(candidates.contains(&"/gcode".to_string()), "{candidates:?}");
        assert!(
            candidates.contains(&"/subscribe".to_string()),
            "{candidates:?}"
        );
        assert!(candidates.contains(&"/quit".to_string()), "{candidates:?}");
        assert_eq!(
            candidates
                .iter()
                .filter(|candidate| *candidate == "/help")
                .count(),
            1,
            "`help` is the window's and the session's, and is offered once: {candidates:?}"
        );

        // Canonical names only: an alias is a second way to spell a line that is
        // already in the list, and `/h` finds `/help` by prefix as it is.
        for alias in [
            "/h",
            "/?",
            "/q",
            "/exit",
            "/sub",
            "/reload_config",
            "/restart_firmware",
        ] {
            assert!(
                !candidates.contains(&alias.to_string()),
                "{alias} in {candidates:?}"
            );
        }

        // Sorted, so the layer reads the way a list reads.
        let mut sorted = candidates.clone();
        sorted.sort();
        assert_eq!(candidates, sorted);
    }

    #[test]
    fn test_a_gcode_line_completes_from_the_printers_own_names() {
        let mut app = app_with_gcode_commands(&["G28", "M104", "M115"]);
        type_line(&mut app, "m1");
        assert_eq!(
            candidates_at(&app),
            ["M104", "M115"],
            "bare names, no slashes"
        );
    }

    #[test]
    fn test_a_local_line_completes_in_gcode_mode_too() {
        // A local command never reaches the printer, so g-code mode is no reason
        // to stop completing one.
        let mut app = app_with_gcode_commands(&["G28", "M104"]);
        type_line(&mut app, "/su");
        assert_eq!(candidates_at(&app), ["/subscribe"]);
    }

    #[test]
    fn test_request_mode_has_no_names_to_offer() {
        let mut app = app_with(Vec::new());
        type_line(&mut app, "info");
        assert!(
            candidates_at(&app).is_empty(),
            "a method is not the window's"
        );

        // A cached command list is not offered either: `M115` is not a method.
        let mut app = app_with_gcode_commands(&["M115"]);
        app.gcode = false;
        type_line(&mut app, "m1");
        assert!(candidates_at(&app).is_empty());
    }

    #[test]
    fn test_the_window_command_list_is_what_the_window_answers() {
        // Completion draws from `WINDOW_COMMANDS` and `window_command` runs the
        // same set. This holds the two spellings together: a command in one and
        // not the other would be either a candidate that does nothing or a
        // command nobody is offered.
        for name in WINDOW_COMMANDS {
            let line = format!("/{name}");
            assert!(App::new().window_command(&line), "{line} is not answered");

            // And each one is its own candidate, with nothing else alongside.
            let mut app = App::new();
            type_line(&mut app, &line);
            assert_eq!(candidates_at(&app), [line.as_str()]);
        }
    }

    #[test]
    fn test_the_printer_command_list_is_asked_for_until_it_is_answered() {
        let mut app = App::new();
        app.gcode = true;
        assert!(
            app.needs_gcode_commands(),
            "nothing has been asked, whatever the state"
        );

        // The status starts at the handshake's one `info` and is only moved by a
        // `webhooks` push, so a printer that was still loading when the window
        // connected keeps saying `startup` here until its first push arrives.
        // Waiting for `ready` would therefore mean not asking during the load —
        // and asking a printer that is not up is not a wait: it refuses at once.
        app.status = Status::Connected {
            state: "startup".to_string(),
            message: "Loading config".to_string(),
        };
        assert!(
            app.needs_gcode_commands(),
            "a printer still loading is asked anyway, and refuses"
        );

        app.status = Status::Connected {
            state: "ready".to_string(),
            message: "Printer is ready".to_string(),
        };
        assert!(
            app.needs_gcode_commands(),
            "ready, in g-code mode, without a list"
        );

        app.gcode_commands = vec!["M115".to_string()];
        app.gcode_commands_asked = true;
        assert!(
            !app.needs_gcode_commands(),
            "the list is kept, so it is asked once"
        );

        // An answer with nothing in it is still an answer: asking again would
        // be a second round trip for the same empty list.
        app.gcode_commands.clear();
        assert!(
            !app.needs_gcode_commands(),
            "an empty answer is still an answer"
        );
        app.gcode = false;
        assert!(
            !app.needs_gcode_commands(),
            "request mode has nowhere to show it"
        );
    }

    #[test]
    fn test_the_candidate_layer_is_drawn_above_the_input_line() {
        let mut app = app_with_gcode_commands(&["M104", "M115", "M140"]);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);
        app.completion_key(KeyCode::Tab); // pick `M104`, so the bar is somewhere

        // Header row 0, the log to row 5, the input line at 6 and the footer at
        // 7: the layer hangs off the input line and covers the log's last rows.
        let rows = render(&app, 40, 8);
        assert!(rows[3].contains("M104"), "{rows:?}");
        assert!(rows[4].contains("M115"), "{rows:?}");
        assert!(rows[5].contains("M140"), "{rows:?}");
        assert!(rows[6].starts_with("gcode> M104"), "{rows:?}");
        assert!(!rows[0].contains("M1"), "the header is above it: {rows:?}");
    }

    #[test]
    fn test_the_candidate_layer_is_left_out_when_the_log_has_no_room() {
        let mut app = app_with_gcode_commands(&["M104", "M115", "M140"]);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);

        // Five rows: the header, two of log, the input line and the footer. The
        // layer would cover the whole log, so it is not drawn — and the
        // candidates are still there, because the picture is not the feature.
        let rows = render(&app, 40, 5);
        assert!(rows.iter().all(|row| !row.contains("M104")), "{rows:?}");
        assert!(app.completion.is_some());

        // One row more and it fits, so it is back.
        let rows = render(&app, 40, 6);
        assert!(rows[1].contains("M104"), "{rows:?}");
    }

    #[test]
    fn test_the_candidate_layer_covers_the_log_behind_it() {
        let mut app = app_with(vec![Entry::notice(Notice::Info, "XXXXXXXXXXXXXXXXXXXX")]);
        app.gcode = true;
        app.gcode_commands = vec!["M104".to_string(), "M115".to_string()];
        app.gcode_commands_asked = true;
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);

        // A short log sits at the top of its pane, so the layer's two rows are
        // below the log's one line — and that line is not showing through them.
        let rows = render(&app, 30, 6);
        assert!(
            rows[1].contains("XXXX"),
            "the log line is above it: {rows:?}"
        );
        assert!(rows[2].contains("M104"), "{rows:?}");
        assert!(rows[3].contains("M115"), "{rows:?}");
        assert!(
            rows[2..=3].iter().all(|row| !row.contains("XXX")),
            "the log is cleared behind the layer: {rows:?}"
        );
    }

    #[test]
    fn test_the_picked_candidate_is_the_highlighted_row() {
        let mut app = app_with_gcode_commands(&["M104", "M115", "M140"]);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab);
        app.completion_key(KeyCode::Tab); // pick `M104`

        let mut terminal = Terminal::new(TestBackend::new(40, 8)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let buffer = terminal.backend().buffer();

        assert!(
            buffer[(1, 3)].modifier.contains(Modifier::REVERSED),
            "the picked row is a bar: {:?}",
            buffer[(1, 3)]
        );
        assert!(
            !buffer[(1, 4)].modifier.contains(Modifier::REVERSED),
            "and the others are not: {:?}",
            buffer[(1, 4)]
        );
        assert_eq!(buffer[(2, 4)].symbol(), "1", "still the candidates' text");
    }

    #[test]
    fn test_a_long_candidate_list_scrolls_to_keep_the_pick_in_view() {
        let names: Vec<String> = (100..110).map(|number| format!("M{number}")).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        let mut app = app_with_gcode_commands(&names);
        type_line(&mut app, "m1");
        app.completion_key(KeyCode::Tab); // opens on `M10`, the candidates' prefix
        for _ in 0..10 {
            app.completion_key(KeyCode::Tab);
        }
        assert_eq!(app.input.text(), "M109", "the last of the ten");

        // Six rows for ten candidates: the list has to have moved for the pick
        // to be in it.
        let rows = render(&app, 40, 9);
        let text = rows.join("\n");
        assert!(text.contains("M109"), "{text}");
        assert!(
            !text.contains("M100"),
            "the front has scrolled away: {text}"
        );
        assert!(rows[7].starts_with("gcode> M109"), "{rows:?}");
    }
}
