//! Running the external programs the two services need: `npm`, `python3`, `pip`.
//!
//! The preparation step of both subcommands is "run this program, look at what it
//! printed, and stop if it failed". That is all this seam is: an [`Invocation`]
//! (program, arguments, working directory) in, a [`Completed`] (exit status and
//! both streams) out. The real implementation is [`SystemRunner`]; the tests
//! substitute a fake that records what would have run and answers with scripted
//! output, so no test installs a package or builds a website.

use std::path::PathBuf;
use std::process::Command;

/// One external program invocation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Invocation {
    /// The program, as named on `PATH` or as an absolute path.
    pub program: String,
    /// Its arguments, in order.
    pub args: Vec<String>,
    /// The directory to run it in, or `None` for this process's own.
    pub cwd: Option<PathBuf>,
}

impl Invocation {
    /// An invocation of `program` with `args`, run in this process's directory.
    pub fn new<I, S>(program: &str, args: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        Invocation {
            program: program.to_string(),
            args: args.into_iter().map(Into::into).collect(),
            cwd: None,
        }
    }

    /// The same invocation, but run in `cwd`.
    pub fn in_dir(mut self, cwd: impl Into<PathBuf>) -> Self {
        self.cwd = Some(cwd.into());
        self
    }

    /// The invocation as an operator would type it, for log lines and errors.
    pub fn command_line(&self) -> String {
        let mut line = quote(&self.program);
        for arg in &self.args {
            line.push(' ');
            line.push_str(&quote(arg));
        }
        line
    }
}

/// One argument as a shell would need it quoted, for a line meant to be typed.
///
/// Only spaces are quoted: a printed command line is for reading and for
/// copying into a shell, and quoting every shell metacharacter would make the
/// ordinary path unreadable.
pub fn quote(argument: &str) -> String {
    if argument.contains(' ') {
        format!("\"{argument}\"")
    } else {
        argument.to_string()
    }
}

/// What one invocation produced.
#[derive(Clone, Debug, Default)]
pub struct Completed {
    /// The exit status.
    pub status: i32,
    /// Everything it wrote to stdout.
    pub stdout: String,
    /// Everything it wrote to stderr.
    pub stderr: String,
}

impl Completed {
    /// Whether the program exited successfully.
    pub fn success(&self) -> bool {
        self.status == 0
    }

    /// The last few lines of stderr, for an error message.
    ///
    /// A failing `npm` or `pip` explains itself at the end of its output; the
    /// beginning is progress noise. Twenty lines is enough to carry the reason
    /// without pasting a build log into a one-line error.
    pub fn stderr_tail(&self) -> String {
        const KEEP: usize = 20;
        let lines: Vec<&str> = self
            .stderr
            .lines()
            .filter(|line| !line.trim().is_empty())
            .collect();
        let start = lines.len().saturating_sub(KEEP);
        lines[start..].join("\n")
    }
}

/// Runs external programs.
///
/// A single method, because that is the whole contract the services need: the
/// tests only ever care *which* program with *which* arguments would run, and
/// whether a failure stops the service.
pub trait CommandRunner {
    /// Run `invocation` to completion and collect its output.
    fn run(&self, invocation: &Invocation) -> std::io::Result<Completed>;
}

/// The real runner: whatever is on `PATH`.
pub struct SystemRunner;

impl CommandRunner for SystemRunner {
    fn run(&self, invocation: &Invocation) -> std::io::Result<Completed> {
        let mut command = Command::new(&invocation.program);
        command.args(&invocation.args);
        if let Some(cwd) = &invocation.cwd {
            command.current_dir(cwd);
        }
        let output = command.output()?;
        Ok(Completed {
            status: output.status.code().unwrap_or(-1),
            stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        })
    }
}

/// Why a program that had to run could not have its output collected, spelled
/// out with the command line that was tried.
///
/// The usual cause is the program not being installed at all (`ErrorKind::
/// NotFound`), which a bare OS error does not say.
pub fn start_error(invocation: &Invocation, error: &std::io::Error) -> String {
    format!("cannot run `{}`: {error}", invocation.command_line())
}

/// The error for a program that ran and failed.
pub fn failure_error(invocation: &Invocation, completed: &Completed) -> String {
    let tail = completed.stderr_tail();
    if tail.is_empty() {
        format!(
            "`{}` failed (exit {}) with no error output",
            invocation.command_line(),
            completed.status
        )
    } else {
        format!(
            "`{}` failed (exit {}):\n{}",
            invocation.command_line(),
            completed.status,
            tail
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_line_quotes_arguments_with_spaces() {
        let invocation = Invocation::new("npm", ["run", "build", "--prefix", "/a b"]);
        assert_eq!(invocation.command_line(), "npm run build --prefix \"/a b\"");
    }

    #[test]
    fn stderr_tail_keeps_the_end() {
        let mut completed = Completed::default();
        completed.stderr = (0..30)
            .map(|i| format!("line {i}"))
            .collect::<Vec<_>>()
            .join("\n");
        let tail = completed.stderr_tail();
        assert!(tail.starts_with("line 10"), "tail was:\n{tail}");
        assert!(tail.ends_with("line 29"));
    }
}
