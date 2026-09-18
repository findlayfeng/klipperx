//! The plain front-end: `klippy-client console` when there is no terminal.
//!
//! This is what a pipe gets — `printf 'list_endpoints\n' | klippy-client console -a …`
//! — and what `--plain` asks for on a terminal. Output is one line per event and
//! input is read line by line, so the result is a log that can be redirected,
//! grepped or pasted from, which a full-screen window deliberately is not.
//!
//! It shares everything but the drawing with [`tui`](super::tui): both drive the
//! same [`Session`] and render the same [`Entry`] values. What differs is one
//! line versus a window, and that a pipe has no prompt to fight with — which is
//! why pushes arriving here are simply printed in place.
//!
//! # Not covered
//!
//! * No reconnection, as in the window: the session ends when the server goes
//!   away, with the reason printed.
//! * `Sent` entries are not echoed. The prompt line already showed them, and a
//!   pipe's reader is interested in the answers.

use std::io::{IsTerminal as _, Write as _};
use std::time::Duration;

use tokio::io::AsyncBufReadExt;

use klippy_api::address::ApiTarget;
use klippy_api::TransportError;

use crate::session::{Control, Entry, Notice, Output, Session};

/// How long to keep reading after input ends, so replies already on their way
/// are printed before the session ends.
pub const LEAVE_GRACE: Duration = Duration::from_secs(1);

/// Run the plain session until input ends or the server goes away.
///
/// # Errors
/// Returns [`TransportError`] if the server cannot be reached or goes away.
pub async fn run(target: ApiTarget) -> Result<(), TransportError> {
    let mut session = Session::connect(target).await?;
    let mut out = Stdout::new(std::io::stdin().is_terminal());
    out.write(Entry::notice(
        Notice::Info,
        format!("Connected to {}.", session.target()),
    ));

    session.handshake(&mut out).await?;

    let mut lines = tokio::io::BufReader::new(tokio::io::stdin()).lines();
    loop {
        if out.prompt {
            print!("klippy> ");
            let _ = std::io::stdout().flush();
        }

        tokio::select! {
            line = lines.next_line() => match line {
                // End of input: the user pressed ^D, or a pipe ran out.
                Ok(None) => break,
                Err(err) => {
                    out.write(Entry::notice(Notice::Problem, format!("cannot read input: {err}")));
                    break;
                }
                Ok(Some(line)) => match session.handle_line(&line, &mut out).await? {
                    Control::Continue => (),
                    Control::Quit => break,
                },
            },
            // A push, or a late reply. Printed as it arrives, which is what lets
            // a subscription be watched without typing anything.
            message = session.receive() => match message {
                Ok(message) => out.write(message.into()),
                Err(err) => {
                    out.write(Entry::notice(Notice::Failure, err.to_string()));
                    break;
                }
            },
        }
    }

    // Leaving is not the same as abandoning: replies to what was just typed are
    // still owed, and a pipe delivers every line before the server has seen the
    // first one.
    session.drain(&mut out, LEAVE_GRACE).await;
    out.write(Entry::notice(
        Notice::Info,
        format!("Disconnected from {}.", session.target()),
    ));
    Ok(())
}

/// Writes entries to stdout, one line each.
struct Stdout {
    /// Whether to show `klippy> ` — only when a person is watching.
    prompt: bool,
}

impl Stdout {
    fn new(prompt: bool) -> Self {
        Self { prompt }
    }
}

impl Output for Stdout {
    fn write(&mut self, entry: Entry) {
        // Sent entries are the prompt line's business, not the log's.
        if matches!(entry, Entry::Sent { .. }) {
            return;
        }
        println!("{}", entry.text());
    }
}
