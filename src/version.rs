//! The version this host reports as its own.
//!
//! Upstream computes `software_version` **at runtime** from its checkout: `main()`
//! takes `util.get_git_version()["version"]` and stores it in `start_args`
//! (`third_party/klipper/klippy/klippy.py:310-335`), and that helper runs
//! `git -C <klippy/..> describe --always --tags --long --dirty`
//! (`third_party/klipper/klippy/util.py:195-225`). Those four flags are what make
//! the field look like `v0.12.0-123-gabcdef`: `<tag>-<commits since the tag>-g<sha>`,
//! always with the `-<n>-g<sha>` part (`--long`), degrading to a bare sha when no
//! tag is reachable (`--always`), and taking `-dirty` when the tree is modified.
//! The same describe feeds `M115` (`gcode.py:346`), `error_mcu` (`error_mcu.py:97`)
//! and `configfile` (`configfile.py:535`).
//!
//! This host pins the same shape **at compile time** instead, for two reasons:
//!
//! * a worktree carries `third_party/klipper` as an empty directory, so the build
//!   cannot run git over the submodule at all;
//! * the local clone holds **no tags** (`for-each-ref refs/tags` is empty), so a
//!   local `git describe` would fall back to `--always` and report a bare sha —
//!   never `-734`, the field that has to come from upstream.
//!
//! So the upstream half is a constant, derived offline from upstream's refs, and
//! the crate half comes from `CARGO_PKG_VERSION` (unmanaged for now: `0.1.0`).

/// The upstream klipper baseline this host replicates, spelled the way upstream's
/// `git describe --tags --long` would spell it **in the upstream repository**:
/// `<tag>-<commits since the tag>-g<sha>`.
///
/// 2026-10-19 derivation, cross-checked against two independent sources:
///
/// * **base commit** — the parent of this fork's first commit in the submodule:
///   `git -C third_party/klipper rev-parse 54e8a778^` → `fe4eb865…` (the newest
///   upstream-authored commit in the ancestry, `load_cell_probe: Improve precision
///   for slow ADCs (#7301)`, 2026-08-13).
/// * **tag** — upstream's newest tag at the time, `v0.13.0` = `61c0c8d2…`
///   (`docs: Note the release of v0.13.0`), an ancestor of the base:
///   `git -C third_party/klipper merge-base --is-ancestor 61c0c8d2… fe4eb865…`.
/// * **count** — `git -C third_party/klipper rev-list --count 61c0c8d2… fe4eb865…`
///   → `734`; GitHub's read-only `compare/61c0c8d2…fe4eb865` reports
///   `ahead_by: 734` (same number, other side of the wire).
/// * **sha** — `git -C third_party/klipper rev-parse --short=7 fe4eb865…` → `fe4eb86`.
///
/// Recompute it whenever the submodule moves past a new upstream base: redo the
/// four steps above (tag list from upstream, read-only), then update the constant
/// and every example in the manuals that quotes the reported version.
pub const UPSTREAM_KLIPPER_VERSION: &str = "v0.13.0-734-gfe4eb86";

/// What `info`'s `software_version`, `M115` and `error_mcu` report: the upstream
/// baseline, a `-`, and this crate's own version.
pub fn software_version() -> String {
    format!("{UPSTREAM_KLIPPER_VERSION}-{}", env!("CARGO_PKG_VERSION"))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// The upstream half is a `git describe --tags --long` string:
    /// `<v-tag>-<count>-g<7 hex digits>`, so the `-123` field is present and
    /// numeric rather than a placeholder.
    #[test]
    fn the_upstream_half_is_a_git_describe() {
        let parts: Vec<&str> = UPSTREAM_KLIPPER_VERSION.split('-').collect();
        assert_eq!(parts.len(), 3, "{UPSTREAM_KLIPPER_VERSION}");
        assert!(
            parts[0].starts_with('v') && parts[0].contains('.'),
            "the tag: {}",
            parts[0]
        );
        assert!(
            parts[1].parse::<usize>().is_ok(),
            "the commit count is a number: {}",
            parts[1]
        );
        assert_eq!(parts[2].len(), 8, "g + seven hex digits: {}", parts[2]);
        assert!(parts[2].starts_with('g'), "{}", parts[2]);
        assert!(
            parts[2][1..].bytes().all(|byte| byte.is_ascii_hexdigit()),
            "{}",
            parts[2]
        );
    }

    /// The reported version is the upstream baseline plus the crate version,
    /// and it is the string the start arguments (hence `info`) carry.
    #[test]
    fn the_reported_version_is_the_upstream_one_plus_the_crate_one() {
        let crate_version = env!("CARGO_PKG_VERSION");
        assert_eq!(
            software_version(),
            format!("{UPSTREAM_KLIPPER_VERSION}-{crate_version}")
        );
        assert!(software_version().starts_with(UPSTREAM_KLIPPER_VERSION));
        assert!(software_version().ends_with(&format!("-{crate_version}")));

        let args = crate::core::klippy::api::StartArgs::collect(
            "/tmp/printer.cfg",
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        );
        assert_eq!(
            args.software_version,
            software_version(),
            "`info` reports the start arguments' version"
        );
    }
}
