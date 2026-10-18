//! `klipperx mainsail` and `klipperx moonraker`: the web UI and the API server it
//! talks to, run as local services.
//!
//! The two subcommands are one feature in two halves, and deliberately symmetric:
//! each takes a checkout that is not a Rust build (a Vite site; a Python
//! application), prepares it on first run — `npm ci && npm run build` for the
//! site, a virtualenv with `pip install -r` for Moonraker — and then serves it.
//! Later runs re-check the preparation instead of repeating it, so the second
//! `klipperx mainsail` starts in milliseconds rather than in minutes.
//!
//! Neither half talks to the network on its own account: every external program
//! (`npm`, `python3 -m venv`, `pip`) goes through [`command::CommandRunner`], so
//! the tests drive the preparation logic with a recorded fake instead of
//! installing anything.
//!
//! What is here is the service hosting only. The pieces they serve are the
//! upstream checkouts under `third_party/` — the repository's own CI does not
//! build either of them, and neither does `cargo build`.

pub mod command;
pub mod mainsail;
pub mod moonraker;
mod sha256;

use std::path::{Path, PathBuf};

/// The repository root: the directory `third_party/` and `src/` live in.
///
/// Taken from the crate's own location rather than the current directory, so a
/// subcommand finds its checkouts whichever directory it was started from. In a
/// worktree this is that worktree's root.
pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Expand a leading `~` or `~/…` with `$HOME`.
///
/// The two path defaults that point into the user's home are written as the
/// literal strings clap prints in `--help` (`~/printer_data`), because that is
/// what an operator typed and expects to see back; the expansion happens when
/// the service resolves its paths. A bare `~user` form is not expanded — it is
/// not a path this program invents, and leaving it alone produces a readable
/// "no such directory" instead of a guess.
pub(crate) fn expand_home(path: &Path) -> Result<PathBuf, String> {
    let text = path.to_string_lossy();
    let Some(rest) = text.strip_prefix('~') else {
        return Ok(path.to_path_buf());
    };
    if !(rest.is_empty() || rest.starts_with('/')) {
        return Ok(path.to_path_buf());
    }
    let home = std::env::var_os("HOME")
        .ok_or_else(|| format!("cannot expand {}: HOME is not set", path.display()))?;
    let home = PathBuf::from(home);
    Ok(if rest.is_empty() {
        home
    } else {
        home.join(rest.trim_start_matches('/'))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expand_home_replaces_a_leading_tilde() {
        let home = std::env::var("HOME").expect("HOME is set for tests");
        assert_eq!(
            expand_home(Path::new("~/printer_data")).expect("expanded"),
            PathBuf::from(&home).join("printer_data")
        );
        assert_eq!(
            expand_home(Path::new("~")).expect("expanded"),
            PathBuf::from(&home)
        );
    }

    #[test]
    fn expand_home_leaves_absolute_and_named_paths_alone() {
        assert_eq!(
            expand_home(Path::new("/tmp/x")).expect("unchanged"),
            PathBuf::from("/tmp/x")
        );
        // `~user/…` is not a path this program invents; it stays as typed.
        assert_eq!(
            expand_home(Path::new("~user/x")).expect("unchanged"),
            PathBuf::from("~user/x")
        );
    }
}
