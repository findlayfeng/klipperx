//! Which `host_library:` paths a mux addresses, and the conflicts that make the
//! decision impossible.
//!
//! A `host_library:` path usually belongs to one `[mcu]`: one library, one MCU,
//! frames on the wire exactly as the protocol writes them. When the path is
//! named by more than one section — or by one that writes a `uuid` — the frames
//! must be addressed with the serial mux's per-id header instead, because more
//! than one board answers on the link. This module is the load-time pass that
//! tells the two shapes apart, the same way [`serial_ports`](super::serial_ports)
//! does for a `serial:` path, and the parse layer (`config/mcu.rs`) rejects a
//! path whose MCUs cannot be told apart.
//!
//! The rules, per path:
//!
//! - one section, no `uuid` — **not** muxed: bare frames, exactly as before;
//! - one section, with a `uuid` — muxed;
//! - two or more sections — muxed, and **at most one** of them may omit
//!   `uuid` (it claims the bridge's own device); every other one must write a
//!   `uuid` to be told apart.
//!
//! Two configurations leave two boards indistinguishable and are config errors
//! naming the sections involved: two or more sections on the path all omitting
//! `uuid`, and two sections on the path writing the same `uuid`.
//!
//! Like [`SerialPorts::classify`](super::serial_ports::SerialPorts), this is a
//! pre-scan: it reads a sibling section's raw option text
//! (`section().get_text`, never a tracked getter), so it never records a read on
//! the shared access tracker and thus never exempts a `[mcu]` option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account. The section's own factory is the one that reads and validates
//! `host_library`/`uuid`; a `uuid` that does not parse is left for it to report.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The registry key this pass is looked up and registered under.
pub const HOST_LIBRARIES_OBJECT: &str = "host_libraries";

/// The host-library paths the config names, grouped by path.
///
/// Built once at load by [`HostLibraries::classify`]; the attach layer asks it
/// whether a path is muxed and who is on it before opening the library.
#[derive(Debug, Default)]
pub struct HostLibraries {
    /// path → its `[mcu]` sections. A `BTreeMap` so a conflict is reported in
    /// the same order on every run.
    by_path: BTreeMap<String, PathGroup>,
}

/// One `host_library:` path and the `[mcu]` sections that name it.
#[derive(Debug)]
struct PathGroup {
    /// The MCU names naming the path, in `[mcu]` declaration order: the bare
    /// `[mcu]` first, then every `[mcu <name>]` in file order, the order
    /// `McuIds::reserve_explicit` reads them in.
    members: Vec<String>,
    /// Whether the path's frames carry the mux header ([`HostLibraries::is_mux`]).
    is_mux: bool,
}

impl HostLibraries {
    /// Group every `[mcu]` section's `host_library:` path and decide whether it
    /// is muxed.
    ///
    /// A section without a `host_library` option (serial, CAN, the `test:`
    /// simulator) is not on a host-library path and is skipped. The section list
    /// is the same two families `McuIds::reserve_explicit` scans: the bare
    /// `[mcu]` and every `[mcu <name>]`.
    ///
    /// # Errors
    /// A path whose MCUs cannot be told apart — two or more sections omitting
    /// `uuid`, or two writing the same one. The error names the sections.
    pub fn classify(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        // path → (mcu name, the uuid it writes). A `None` uuid means the section
        // writes none; a uuid that does not parse is still kept as its text, so
        // a duplicate is caught here and the section's own factory reports the
        // bad value.
        let mut groups: BTreeMap<String, Vec<(String, Option<String>)>> = BTreeMap::new();
        for section in config
            .sibling("mcu")
            .into_iter()
            .chain(config.sibling_prefix_sections("mcu"))
        {
            let Some(path) = section.section().get_text("host_library") else {
                continue;
            };
            groups
                .entry(path)
                .or_default()
                .push((section.identifier(), declared_uuid(&section)));
        }

        let mut by_path: BTreeMap<String, PathGroup> = BTreeMap::new();
        for (path, members) in groups {
            let is_mux = path_is_mux(&path, &members)?;
            by_path.insert(
                path,
                PathGroup {
                    members: members.into_iter().map(|(name, _)| name).collect(),
                    is_mux,
                },
            );
        }
        Ok(Self { by_path })
    }

    /// Whether the frames on `path` carry the mux header. A path no `[mcu]`
    /// names is not muxed.
    pub fn is_mux(&self, path: &str) -> bool {
        self.by_path.get(path).is_some_and(|group| group.is_mux)
    }

    /// The MCUs that name `path`, in declaration order — one that addresses its
    /// library with bare frames, two or more that need the mux header. Empty
    /// when no `[mcu]` names it.
    pub fn peers(&self, path: &str) -> &[String] {
        self.by_path
            .get(path)
            .map(|group| group.members.as_slice())
            .unwrap_or(&[])
    }
}

impl PrinterObject for HostLibraries {
    /// The registry reports no status, exactly as the id allocator and the
    /// serial-port grouping define none.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible: nothing here is a status a client can query.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// Reject a `host_library:` path whose MCUs cannot be told apart.
///
/// The parse layer (`config/mcu.rs`) calls this as it parses each host section,
/// so a conflict is a config error and the config fails to load — the same place
/// [`path_is_shared`](super::serial_ports::path_is_shared) answers the shared
/// serial-port question. It runs the same grouping [`HostLibraries::classify`]
/// does and keeps only the verdict; each host section's parse calls it once.
///
/// # Errors
/// As [`HostLibraries::classify`].
pub fn validate(config: &ConfigWrapper) -> Result<(), ConfigError> {
    HostLibraries::classify(config).map(|_| ())
}

/// The single classification, created on first demand.
///
/// The first `[mcu]` factory to load builds it from the whole config and
/// registers it; every later one finds it already there and **does not**
/// re-classify, so the grouping is a load-time constant rather than something
/// each section recomputes. Mirrors [`serial_ports::ensure`](super::serial_ports::ensure);
/// the attach layer calls it to ask whether a library path is muxed.
///
/// # Errors
/// A path conflict ([`HostLibraries::classify`]), or a registry key that is
/// already taken by a different object.
pub fn ensure(
    printer: &Arc<Printer>,
    config: &ConfigWrapper,
) -> Result<Arc<HostLibraries>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<HostLibraries>(HOST_LIBRARIES_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(HostLibraries::classify(config)?);
    printer.add_object(HOST_LIBRARIES_OBJECT, object.clone())?;
    Ok(object)
}

/// Whether the sections naming `path` need the mux header, rejecting the shapes
/// that cannot be told apart.
///
/// One section is muxed exactly when it writes a `uuid`. Two or more are always
/// muxed; with more than one of them omitting `uuid` there is no way to pick
/// which board a bare frame belongs to, and with two writing the same `uuid`
/// there is no way to tell which of them a header names — both are reported with
/// the sections involved.
fn path_is_mux(path: &str, members: &[(String, Option<String>)]) -> Result<bool, ConfigError> {
    let Some((_, first_uuid)) = members.first() else {
        return Ok(false);
    };
    if members.len() == 1 {
        return Ok(first_uuid.is_some());
    }
    let without = members
        .iter()
        .filter(|(_, uuid)| uuid.is_none())
        .map(|(name, _)| name.as_str())
        .collect::<Vec<_>>();
    if without.len() >= 2 {
        return Err(ConfigError::new(format!(
            "host library '{path}': {} name it without a uuid, but at most one MCU on a path \
             may omit it (it claims the bridge's own device)",
            quoted(&without)
        )));
    }
    // Two sections writing one identity name the same board twice: a mux header
    // built from it would address both at once. A `BTreeMap` so the reported
    // pair is stable across runs.
    let mut by_uuid: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for (name, uuid) in members {
        if let Some(uuid) = uuid {
            by_uuid
                .entry(uuid.as_str())
                .or_default()
                .push(name.as_str());
        }
    }
    if let Some((uuid, names)) = by_uuid.iter().find(|(_, names)| names.len() >= 2) {
        return Err(ConfigError::new(format!(
            "host library '{path}': {} set the same uuid {uuid}, so the boards cannot be \
             told apart",
            quoted(names)
        )));
    }
    Ok(true)
}

/// The `uuid` a `[mcu]` section writes for its host library: the canonical
/// `uuid` or its `canbus_uuid` alias, trimmed and lowercased so two spellings of
/// one identity compare equal, or `None` when it writes neither. Read raw so the
/// scan records no access (see the module docs).
fn declared_uuid(section: &ConfigWrapper) -> Option<String> {
    section
        .section()
        .get_text("uuid")
        .or_else(|| section.section().get_text("canbus_uuid"))
        .map(|text| text.trim().to_ascii_lowercase())
}

/// The section names as a quoted, comma-separated list for an error message.
fn quoted(names: &[&str]) -> String {
    names
        .iter()
        .map(|name| format!("'{name}'"))
        .collect::<Vec<_>>()
        .join(", ")
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::Config;

    /// Parse `text` and classify through its first `[mcu]` section.
    ///
    /// Any `[mcu]`-family wrapper reaches the same sibling sections, so which one
    /// is wrapped does not change the answer.
    fn classify(text: &str) -> Result<HostLibraries, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        HostLibraries::classify(&wrapper)
    }

    const LIB: &str = "/lib/libklipper_host.so";

    /// One section, no `uuid`: bare frames, today's behaviour.
    #[test]
    fn a_single_section_without_a_uuid_is_not_muxed() {
        let libraries = classify(&format!("[mcu]\nhost_library: {LIB}\n")).unwrap();
        assert!(!libraries.is_mux(LIB));
        assert_eq!(libraries.peers(LIB), ["mcu"]);
    }

    /// One section with a `uuid` is muxed: the board is addressed by its identity
    /// even alone.
    #[test]
    fn a_single_section_with_a_uuid_is_muxed() {
        let libraries =
            classify(&format!("[mcu]\nhost_library: {LIB}\nuuid: 11aa22bb33cc\n")).unwrap();
        assert!(libraries.is_mux(LIB));
    }

    /// The `canbus_uuid` alias carries the same identity as `uuid`, so it marks
    /// the path muxed too.
    #[test]
    fn a_single_section_with_the_canbus_uuid_alias_is_muxed() {
        let libraries = classify(&format!(
            "[mcu]\nhost_library: {LIB}\ncanbus_uuid: 11aa22bb33cc\n"
        ))
        .unwrap();
        assert!(libraries.is_mux(LIB));
    }

    /// Two sections on one path are muxed: the one without a `uuid` claims the
    /// bridge's own device, the other names its board.
    #[test]
    fn two_sections_are_muxed_and_one_may_omit_the_uuid() {
        let libraries = classify(&format!(
            "[mcu]\nhost_library: {LIB}\n\
             [mcu tool]\nhost_library: {LIB}\nuuid: 22bb33cc44dd\n"
        ))
        .unwrap();
        assert!(libraries.is_mux(LIB));
        assert_eq!(libraries.peers(LIB), ["mcu", "mcu tool"]);
    }

    /// Two sections writing distinct uuids are muxed as well.
    #[test]
    fn two_sections_with_distinct_uuids_are_muxed() {
        let libraries = classify(&format!(
            "[mcu a]\nhost_library: {LIB}\nuuid: 11aa22bb33cc\n\
             [mcu b]\nhost_library: {LIB}\nuuid: 22bb33cc44dd\n"
        ))
        .unwrap();
        assert!(libraries.is_mux(LIB));
    }

    /// A path no `[mcu]` names is not muxed and has no peers.
    #[test]
    fn a_path_no_mcu_names_is_not_muxed() {
        let libraries = classify("[mcu]\nserial: /dev/ttyACM0\n").unwrap();
        assert!(!libraries.is_mux(LIB));
        assert!(libraries.peers(LIB).is_empty());
    }

    /// Two sections that both omit `uuid` cannot be told apart: a config error
    /// naming both.
    #[test]
    fn two_sections_without_a_uuid_are_a_config_error() {
        let err = classify(&format!(
            "[mcu]\nhost_library: {LIB}\n\
             [mcu tool]\nhost_library: {LIB}\n"
        ))
        .unwrap_err();
        assert!(err.to_string().contains("host library"), "{err}");
        assert!(err.to_string().contains("'mcu'"), "{err}");
        assert!(err.to_string().contains("'mcu tool'"), "{err}");
    }

    /// Two sections writing the same `uuid` name one board twice: a config error
    /// naming both. The spellings differ in case, and the identity is compared
    /// case-insensitively.
    #[test]
    fn two_sections_with_the_same_uuid_are_a_config_error() {
        let err = classify(&format!(
            "[mcu a]\nhost_library: {LIB}\nuuid: 11aa22bb33cc\n\
             [mcu b]\nhost_library: {LIB}\ncanbus_uuid: 11AA22BB33CC\n"
        ))
        .unwrap_err();
        assert!(err.to_string().contains("'mcu a'"), "{err}");
        assert!(err.to_string().contains("'mcu b'"), "{err}");
        assert!(err.to_string().contains("11aa22bb33cc"), "{err}");
    }

    /// `validate` is the same verdict the parse layer calls: a conflict is an
    /// error, agreement is not.
    #[test]
    fn validate_reports_the_same_conflict() {
        let (config, _) = Config::from_text(&format!(
            "[mcu]\nhost_library: {LIB}\n\
             [mcu tool]\nhost_library: {LIB}\n"
        ))
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        assert!(validate(&wrapper).is_err());
    }

    /// The scan records no reads: it must not exempt a sibling `[mcu]` section's
    /// options from `check_unused` on its own account — that is the section's own
    /// factory's job.
    #[test]
    fn classify_records_no_reads() {
        let (config, _) = Config::from_text(&format!(
            "[mcu]\nhost_library: {LIB}\nuuid: 11aa22bb33cc\n\
             [mcu tool]\nhost_library: {LIB}\nuuid: 22bb33cc44dd\n"
        ))
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        HostLibraries::classify(&wrapper).expect("distinct uuids agree");

        assert!(
            access.sections().is_empty(),
            "classify read {:?}, which would exempt them from check_unused",
            access.sections()
        );
    }

    /// A wrapper that cannot reach its siblings sees no libraries: a test's small
    /// wrapper, or a part reading a stored section back.
    #[test]
    fn a_wrapper_without_the_config_sees_no_libraries() {
        let (config, _) =
            Config::from_text(&format!("[mcu]\nhost_library: {LIB}\n")).expect("parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::untracked(section);
        let libraries = HostLibraries::classify(&wrapper).expect("nothing to conflict");
        assert!(libraries.peers(LIB).is_empty());
        assert!(!libraries.is_mux(LIB));
    }
}
