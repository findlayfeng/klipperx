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

/// The Mainsail checkout: the Vite project `npm` runs in.
pub fn mainsail_checkout() -> PathBuf {
    repo_root().join("third_party").join("mainsail")
}

/// The built Mainsail site of the checkout layer: `<checkout>/dist`.
///
/// `KLIPPERX_MAINSAIL_DIST` overrides it with another built site — the same
/// override pattern as `KLIPPERX_KLIPPER_DIR`, and the same job: pointing a
/// worktree (or a packaged build) at a `dist/` that lives elsewhere. It moves
/// only what is served; `npm`, when it has to run, still runs in the checkout.
/// `build.rs` reads the same variable when it embeds a site.
pub fn mainsail_dist() -> PathBuf {
    override_path("KLIPPERX_MAINSAIL_DIST", || {
        mainsail_checkout().join("dist")
    })
}

/// The Moonraker checkout: the Python sources `moonraker/moonraker.py` lives in.
///
/// `KLIPPERX_MOONRAKER_DIR` overrides it, like `KLIPPERX_KLIPPER_DIR` does for
/// the klipper submodule: a worktree carries no submodules, and a packaged
/// build may ship the sources somewhere else entirely.
pub fn moonraker_checkout() -> PathBuf {
    override_path("KLIPPERX_MOONRAKER_DIR", || {
        repo_root().join("third_party").join("moonraker")
    })
}

/// The Moonraker copy kept beside the venv: `$XDG_DATA_HOME/klipperx/moonraker`
/// (`~/.local/share/klipperx/moonraker` when `XDG_DATA_HOME` is unset — the same
/// base directory the default venv lives under).
///
/// This is the layer packaging owns: a package can put Moonraker's sources here
/// and `klipperx moonraker` runs them, with no checkout and no code change. On a
/// first run without one, the checkout is copied in.
pub fn bundled_moonraker() -> Result<PathBuf, String> {
    bundled_source_dir(std::env::var_os("XDG_DATA_HOME"), std::env::var_os("HOME"))
}

/// [`bundled_moonraker`] with the environment read already done, so the
/// resolution itself is testable without mutating process-wide state.
fn bundled_source_dir(
    xdg_data_home: Option<std::ffi::OsString>,
    home: Option<std::ffi::OsString>,
) -> Result<PathBuf, String> {
    let base = match xdg_data_home.filter(|value| !value.is_empty()) {
        Some(directory) => PathBuf::from(directory),
        None => {
            let home = home.filter(|value| !value.is_empty()).ok_or_else(|| {
                "cannot find the data directory: neither XDG_DATA_HOME nor HOME is set".to_string()
            })?;
            PathBuf::from(home).join(".local").join("share")
        }
    };
    Ok(base.join("klipperx").join("moonraker"))
}

/// The value of `variable`, or what `default` builds when it is unset or empty.
fn override_path(variable: &str, default: impl FnOnce() -> PathBuf) -> PathBuf {
    override_path_from(std::env::var_os(variable), default)
}

/// [`override_path`] with the value read already, for tests.
fn override_path_from(
    value: Option<std::ffi::OsString>,
    default: impl FnOnce() -> PathBuf,
) -> PathBuf {
    match value.filter(|value| !value.is_empty()) {
        Some(directory) => PathBuf::from(directory),
        None => default(),
    }
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

    #[test]
    fn the_bundled_copy_lives_under_the_data_directory() {
        let xdg = bundled_source_dir(Some("/xdg".into()), Some("/home/u".into()))
            .expect("XDG_DATA_HOME wins");
        assert_eq!(xdg, PathBuf::from("/xdg/klipperx/moonraker"));

        let local = bundled_source_dir(None, Some("/home/u".into())).expect("HOME is the fallback");
        assert_eq!(
            local,
            PathBuf::from("/home/u/.local/share/klipperx/moonraker")
        );

        // An empty XDG_DATA_HOME is the unset one, not a directory named "".
        let empty = bundled_source_dir(Some("".into()), Some("/home/u".into()))
            .expect("HOME is the fallback");
        assert_eq!(
            empty,
            PathBuf::from("/home/u/.local/share/klipperx/moonraker")
        );

        assert!(bundled_source_dir(None, None).is_err());
    }

    #[test]
    fn an_override_replaces_the_default_only_when_it_has_a_value() {
        let default = || PathBuf::from("/default");
        assert_eq!(
            override_path("KLIPPERX_UI_TEST_PATH_NOT_SET", default),
            PathBuf::from("/default")
        );
        assert_eq!(
            override_path_from(Some("/elsewhere".into()), default),
            PathBuf::from("/elsewhere")
        );
        assert_eq!(
            override_path_from(Some("".into()), default),
            PathBuf::from("/default")
        );
    }
}
