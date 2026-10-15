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
//! - one section, no `uuid`, **not a bridge** — **not** muxed: bare frames,
//!   exactly as before;
//! - one section, with a `uuid` — muxed;
//! - one section that a `[uart_device]`/`[can_device]` names as its bridge —
//!   muxed: a bridge's link carries its downstream devices' frames too, so the
//!   path must be addressed by id even though one `[mcu]` names it (rule 7,
//!   `bridge_ports::bridge_mcus`); one or more downstream devices on that
//!   bridge (`bridge_ports::bridge_downstream_counts`) is rule 7's count, kept
//!   as a separate reason **beside** being a bridge rather than instead of it —
//!   a muxed bridge frames its output whatever is hanging on it, so the
//!   zero-device case is a deliberate conservative reading;
//! - two or more sections — muxed, and **at most one** of them may omit
//!   `uuid` (it claims the bridge's own device); every other one must write a
//!   `uuid` to be told apart.
//!
//! Two configurations leave two boards indistinguishable and are config errors
//! naming the sections involved: two or more sections on the path all omitting
//! `uuid`, and two sections on the path writing the same `uuid`. The grouping
//! and the uuid-distinguishability judgment behind those two rules live in
//! `mcu/link_grouping.rs`, shared with the port grouping — one judgment for
//! both link kinds; the wording and the muxed/not verdict stay here.
//!
//! The same pass gives every `host_library:` path an **identity of its own**
//! ([`HostLibraries::identity`]): the uuid the library's own device answers the
//! mux with, which the host assigns because the library is a file it loads and
//! has nothing of its own to derive one from. The identity is a pure function of
//! the path string (so it is per path, not per section and stable across runs),
//! never all zeros (the library's "no identity" value), and steps aside of every
//! uuid the config already names — the `uuid`/`canbus_uuid` of every `[mcu]`,
//! CAN included — so it can never name a board that is already on the link. The
//! [`HostLibraries::named_uuid`] rule then says which uuid an attach writes in
//! its `SET_KLIPPER_NODEID`: the section's own `uuid` when it wrote one (it
//! names the board the bridge routes to), else the path's own identity (the
//! bridge's own device, which only the host knows).
//!
//! One more rule is enforced here, whole-config rather than per path: a config
//! may name **one** `host_library:` path, never two. A klipper host library
//! stops its main loop with a process-wide `SIGUSR1` handler that every library
//! installs over the last one, so two live libraries in one process crash or
//! hang on the way out ([`HostLibraries::reject_a_second_path`]); several
//! sections on the one path are one library and stay allowed. [`validate`] is
//! what turns that into a load-time error.
//!
//! Like [`SerialPorts::classify`](super::serial_ports::SerialPorts), this is a
//! pre-scan: it reads a sibling section's raw option text
//! (`section().get_text`, never a tracked getter), so it never records a read on
//! the shared access tracker and thus never exempts a `[mcu]` option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account. The section's own factory is the one that reads and validates
//! `host_library`/`uuid`; a `uuid` that does not parse is left for it to report.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::mcu::bridge_ports::{bridge_downstream_counts, bridge_mcus};
use crate::core::klippy::mcu::link_grouping::{
    declared_uuid, distinguishability, group_by_key, quoted, Distinguishability, Member,
};
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
    /// The identity the path's own device answers the mux with
    /// ([`HostLibraries::identity`]).
    identity: [u8; 6],
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
        // The `[mcu]` sections' claims on a host-library path: the path they
        // name and the uuid they write. A `None` uuid means the section writes
        // none; a uuid that does not parse is still kept as its text, so a
        // duplicate is caught here and the section's own factory reports the
        // bad value.
        let mut claims: Vec<(String, Member)> = Vec::new();
        // The `[mcu]` names the config makes bridges (port sections' `*_mcu`),
        // read raw like every claim here: a bridge's path is addressed by id
        // even when one section names it (`path_is_mux`).
        let bridges = bridge_mcus(config);
        // Rule 7's count: how many downstream devices each bridge has, read raw
        // like every other claim here.
        let downstream_counts = bridge_downstream_counts(config);
        // Every uuid the config already names, whatever transport carries it, so
        // an assigned identity can never collide with a board that is already
        // addressed by one.
        let mut known: BTreeSet<[u8; 6]> = BTreeSet::new();
        for section in config
            .sibling("mcu")
            .into_iter()
            .chain(config.sibling_prefix_sections("mcu"))
        {
            if let Some(uuid) = declared_uuid_bytes(&section) {
                known.insert(uuid);
            }
            let Some(path) = section.section().get_text("host_library") else {
                continue;
            };
            claims.push((
                path,
                Member {
                    identifier: section.identifier(),
                    uuid: declared_uuid(&section),
                },
            ));
        }

        let mut by_path: BTreeMap<String, PathGroup> = BTreeMap::new();
        let mut taken = known;
        for (path, members) in group_by_key(claims) {
            let has_bridge = members
                .iter()
                .any(|member| bridges.contains(member_name(&member.identifier)));
            let downstream = members
                .iter()
                .filter_map(|member| downstream_counts.get(member_name(&member.identifier)))
                .sum();
            let is_mux = path_is_mux(&path, &members, has_bridge, downstream)?;
            let identity = assign_identity(&path, &mut taken);
            by_path.insert(
                path,
                PathGroup {
                    members: members
                        .iter()
                        .map(|member| member.identifier.clone())
                        .collect(),
                    is_mux,
                    identity,
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

    /// The identity the device behind `path` answers the mux with, or `None`
    /// when no `[mcu]` names the path at all.
    ///
    /// It is assigned once per path at load, a pure function of the path
    /// string, so it is per path rather than per section
    /// and the same config always names the same device. Every path gets one,
    /// muxed or not; only an attach that speaks the mux consumes it.
    pub fn identity(&self, path: &str) -> Option<[u8; 6]> {
        self.by_path.get(path).map(|group| group.identity)
    }

    /// The uuid an attach on `path` names in its `SET_KLIPPER_NODEID`: the
    /// section's `declared` uuid when it wrote one — it names the board the
    /// bridge routes to — else the path's own identity, which only the host
    /// knows (the bridge's own device). `None` when no `[mcu]` names the path.
    pub fn named_uuid(&self, path: &str, declared: Option<[u8; 6]>) -> Option<[u8; 6]> {
        let group = self.by_path.get(path)?;
        Some(declared.unwrap_or(group.identity))
    }

    /// Reject a config that names more than one `host_library:` path.
    ///
    /// A process holds **one** host library: klipper stops its main loop with a
    /// process-wide `SIGUSR1` handler that each library installs over the last
    /// one (`third_party/klipper/src/host/host.c`), so two live libraries in one
    /// process meet either a jump into unloaded code when the first is dropped
    /// (SIGSEGV) or a join whose signal another library consumed (a hang). Two
    /// paths are therefore a config error **naming both**; one path that several
    /// sections name — a mux, or a bridge's downstream references — is **one**
    /// library and stays allowed.
    ///
    /// # Errors
    /// A config whose `[mcu]` sections name two or more different paths.
    fn reject_a_second_path(&self) -> Result<(), ConfigError> {
        if self.by_path.len() < 2 {
            return Ok(());
        }
        let paths: Vec<&str> = self.by_path.keys().map(String::as_str).collect();
        Err(ConfigError::new(format!(
            "host library: one host library per process, but the config names {}. klipper \
             stops its main loop through a process-wide SIGUSR1 handler that each library \
             installs over the last one, so two libraries in one process meet either a jump \
             into unloaded code (a crash) or a join whose signal another library consumed (a \
             hang); name one path from several sections, or run the firmware images in \
             separate processes",
            quoted(&paths)
        )))
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

/// Reject a `host_library:` path whose MCUs cannot be told apart, and a config
/// naming more than one `host_library:` path.
///
/// The parse layer (`config/mcu.rs`) calls this as it parses each host section,
/// so a conflict is a config error and the config fails to load — the same place
/// [`path_is_shared`](super::serial_ports::path_is_shared) answers the shared
/// serial-port question. It runs the same grouping [`HostLibraries::classify`]
/// does and keeps only the verdict; each host section's parse calls it once.
///
/// # Errors
/// As [`HostLibraries::classify`], and a config whose sections name two or more
/// different paths ([`HostLibraries::reject_a_second_path`]).
pub fn validate(config: &ConfigWrapper) -> Result<(), ConfigError> {
    HostLibraries::classify(config)?.reject_a_second_path()
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
/// One section is muxed when it writes a `uuid`, **or** when it is a bridge
/// (`has_bridge`), **or** when one or more downstream devices hang on that
/// bridge (`downstream >= 1`): a bridge's link carries its downstream devices'
/// frames, so the path is addressed by id even alone. The `downstream` term is
/// rule 7's count written out, kept beside `has_bridge` on purpose: a muxed
/// bridge with zero downstream devices is still addressed by id (conservative
/// reading), and its output is framed accordingly. Two or more are always muxed;
/// with more than one of them omitting `uuid` there is no way to pick
/// which board a bare frame belongs to, and with two writing the same `uuid`
/// there is no way to tell which of them a header names — both are reported with
/// the sections involved.
fn path_is_mux(
    path: &str,
    members: &[Member],
    has_bridge: bool,
    downstream: usize,
) -> Result<bool, ConfigError> {
    let Some(first) = members.first() else {
        return Ok(false);
    };
    if members.len() == 1 {
        return Ok(first.uuid.is_some() || has_bridge || downstream >= 1);
    }
    // At most one member may omit its uuid: it claims the bridge's own device.
    // The verdict is the shared distinguishability judgment
    // (`mcu/link_grouping.rs`); the wording is this link's own.
    match distinguishability(members, 1) {
        None => Ok(true),
        Some(Distinguishability::MissingUuid(without)) => Err(ConfigError::new(format!(
            "host library '{path}': {} name it without a uuid, but at most one MCU on a path \
             may omit it (it claims the bridge's own device)",
            quoted(&names(&without))
        ))),
        // Two sections writing one identity name the same board twice: a mux
        // header built from it would address both at once.
        Some(Distinguishability::DuplicateUuid { uuid, members }) => {
            Err(ConfigError::new(format!(
                "host library '{path}': {} set the same uuid {uuid}, so the boards cannot be \
                 told apart",
                quoted(&names(&members))
            )))
        }
    }
}

/// The `[mcu]` **name** a section identifier carries: the sub, or `mcu` for the
/// bare section — the spelling a port section's `*_mcu` writes.
fn member_name(identifier: &str) -> &str {
    identifier.strip_prefix("mcu ").unwrap_or(identifier)
}

/// Member identifiers as the borrowed list `quoted` joins.
fn names(identifiers: &[String]) -> Vec<&str> {
    identifiers.iter().map(String::as_str).collect()
}

/// The identity the device behind `path` answers the mux with: a pure function
/// of the path string, stepped past every uuid `taken` already names.
///
/// The first candidate is [`identity_seed`] — a hash of the path — so the same
/// path always yields the same identity, whatever sections or order the config
/// around it has. It must be non-zero, the library's "no identity" value, and
/// must not collide with a configured uuid or another path's identity; a
/// collision steps to the next value. `taken` is a `BTreeSet`, so the step is
/// deterministic.
fn assign_identity(path: &str, taken: &mut BTreeSet<[u8; 6]>) -> [u8; 6] {
    let mut identity = identity_seed(path);
    while identity == [0; 6] || taken.contains(&identity) {
        identity = next_identity(identity);
    }
    taken.insert(identity);
    identity
}

/// The first identity candidate for `path`: the low six bytes of an FNV-1a hash
/// of the path string.
///
/// FNV-1a is small and dependency-free, and its avalanche is enough here: the
/// value only has to spread a handful of paths across 2^48, not resist an
/// adversary. The low six bytes are taken so the all-zero value stays possible
/// (and is stepped past by [`assign_identity`]) rather than being excluded
/// inside the hash.
fn identity_seed(path: &str) -> [u8; 6] {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in path.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    let mut identity = [0u8; 6];
    identity.copy_from_slice(&hash.to_le_bytes()[..6]);
    identity
}

/// The identity after `identity`, as a big-endian six-byte counter that wraps.
fn next_identity(mut identity: [u8; 6]) -> [u8; 6] {
    for byte in identity.iter_mut().rev() {
        let (next, carry) = byte.overflowing_add(1);
        *byte = next;
        if !carry {
            break;
        }
    }
    identity
}

/// The `uuid`/`canbus_uuid` a `[mcu]` section writes, parsed to its six bytes,
/// or `None` when it writes neither or writes one that does not parse.
///
/// Read raw, like [`declared_uuid`], so the scan records no access. A uuid that
/// does not parse is left out of the known set rather than reported: the
/// section's own factory rejects the config for it.
fn declared_uuid_bytes(section: &ConfigWrapper) -> Option<[u8; 6]> {
    let text = section
        .section()
        .get_text("uuid")
        .or_else(|| section.section().get_text("canbus_uuid"))?;
    parse_uuid_hex(text.trim())
}

/// Six bytes as twelve hex digits, the spelling a `[mcu] uuid:` option uses.
fn parse_uuid_hex(text: &str) -> Option<[u8; 6]> {
    if text.len() != 12 {
        return None;
    }
    let mut uuid = [0u8; 6];
    for (i, byte) in uuid.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(uuid)
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

    /// A lone section a port section names as its **bridge** is muxed: the
    /// bridge's downstream devices' frames travel the path, so it is addressed
    /// by id even though one `[mcu]` names it (rule 7).
    #[test]
    fn a_single_bridge_section_is_muxed() {
        let libraries = classify(&format!(
            "[mcu bridge]\nhost_library: {LIB}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n"
        ))
        .unwrap();
        assert!(libraries.is_mux(LIB));
    }

    /// Rule 7's count, from the same table the serial grouping uses: a lone
    /// section whose bridge carries a downstream `[mcu]` is muxed.
    #[test]
    fn a_lone_bridge_section_with_a_downstream_device_is_muxed() {
        let libraries = classify(&format!(
            "[mcu bridge]\nhost_library: {LIB}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [mcu child]\nuart: uart0\n"
        ))
        .unwrap();
        assert!(libraries.is_mux(LIB));
    }

    /// A lone section no port section names as its bridge stays bare frames:
    /// only being a bridge makes an otherwise-exclusive path muxed.
    #[test]
    fn a_single_section_no_port_names_is_not_muxed() {
        let libraries = classify(&format!(
            "[mcu bridge]\nhost_library: {LIB}\n\
             [uart_device uart0]\nuart_mcu: other\nuart_oid: uart0\n"
        ))
        .unwrap();
        assert!(!libraries.is_mux(LIB));
    }

    /// The bare `[mcu]` is named as a bridge by its `mcu` spelling: the bridge
    /// scan and the member names share that spelling.
    #[test]
    fn the_bare_mcu_spelling_names_a_bridge() {
        let libraries = classify(&format!(
            "[mcu]\nhost_library: {LIB}\n\
             [can_device can0]\ncan_mcu: mcu\ncan_oid: can0\n"
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

    /// A config naming two different `host_library:` paths is refused, and the
    /// message names both and says why: a process holds one host library.
    #[test]
    fn validate_rejects_a_second_host_library_path() {
        let (config, _) = Config::from_text(
            "[mcu]\nhost_library: /lib/one.so\n\
             [mcu tool]\nhost_library: /lib/two.so\n",
        )
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        let err = validate(&wrapper).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("one host library per process"), "{text}");
        assert!(text.contains("'/lib/one.so'"), "{text}");
        assert!(text.contains("'/lib/two.so'"), "{text}");
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
        assert_eq!(libraries.identity(LIB), None);
    }

    // -----------------------------------------------------------------------
    // The identity each path is given (`HostLibraries::identity`).
    // -----------------------------------------------------------------------

    /// Classify `text` through the section `identifier`, which the loader's
    /// wrapper always can reach the whole config through.
    fn classify_from(text: &str, identifier: &str) -> Result<HostLibraries, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config.get_section(identifier).expect("the named section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        HostLibraries::classify(&wrapper)
    }

    /// The twelve hex digits a uuid is written as, for building a config that
    /// names it.
    fn hex(uuid: [u8; 6]) -> String {
        uuid.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// Every path gets a non-zero identity, distinct across paths, muxed or not.
    #[test]
    fn every_path_gets_its_own_nonzero_identity() {
        let libraries = classify_from(
            "[mcu a]\nhost_library: /lib/a.so\n\
             [mcu b]\nhost_library: /lib/b.so\n",
            "mcu a",
        )
        .unwrap();
        let a = libraries.identity("/lib/a.so").expect("a has one");
        let b = libraries.identity("/lib/b.so").expect("b has one");
        assert_ne!(a, [0; 6], "all zeros is the library's unnamed value");
        assert_ne!(b, [0; 6]);
        assert_ne!(a, b, "two paths are two devices");
        assert_eq!(libraries.identity("/lib/c.so"), None, "no section names it");
    }

    /// The identity is a function of the path string alone: the same path in two
    /// configs gets the same identity, however the sections around it are named.
    #[test]
    fn identity_is_a_function_of_the_path_string() {
        let one = classify_from("[mcu x]\nhost_library: /lib/a.so\n", "mcu x").unwrap();
        let two = classify_from("[mcu y]\nhost_library: /lib/a.so\n", "mcu y").unwrap();
        assert_eq!(one.identity("/lib/a.so"), two.identity("/lib/a.so"));
    }

    /// The identity never names a board the config already names: a configured
    /// uuid equal to the path's natural candidate steps it aside.
    #[test]
    fn an_assigned_identity_steps_past_a_configured_uuid() {
        let natural = classify_from("[mcu]\nhost_library: /lib/a.so\n", "mcu")
            .unwrap()
            .identity("/lib/a.so")
            .expect("the path has one");
        // A CAN MCU claims exactly the identity the path would have taken.
        let libraries = classify_from(
            &format!(
                "[mcu]\nhost_library: /lib/a.so\n\
                 [mcu can]\nuuid: {}\n",
                hex(natural)
            ),
            "mcu",
        )
        .unwrap();
        let identity = libraries.identity("/lib/a.so").expect("still assigned");
        assert_ne!(identity, natural, "the configured uuid is taken");
        assert_ne!(identity, [0; 6]);
    }

    /// A host section's own `uuid` is a configured uuid too: the path's identity
    /// must not be it, or the bridge's own device would be the board it routes.
    #[test]
    fn a_host_identity_never_equals_its_sections_uuid() {
        let natural = classify_from("[mcu]\nhost_library: /lib/a.so\n", "mcu")
            .unwrap()
            .identity("/lib/a.so")
            .expect("the path has one");
        let declared = hex(natural);
        let libraries = classify_from(
            &format!("[mcu]\nhost_library: /lib/a.so\nuuid: {declared}\n"),
            "mcu",
        )
        .unwrap();
        assert_ne!(libraries.identity("/lib/a.so"), Some(natural));
        assert!(libraries.is_mux("/lib/a.so"));
    }

    /// Two paths never share an identity even when one's natural candidate is
    /// already named by the config: the assignment steps aside, and both paths
    /// still end up with different, unclaimed values.
    #[test]
    fn two_paths_never_share_an_identity() {
        let natural = classify_from("[mcu a]\nhost_library: /lib/a.so\n", "mcu a")
            .unwrap()
            .identity("/lib/a.so")
            .expect("assigned");
        let taken = hex(natural);
        let libraries = classify_from(
            &format!(
                "[mcu a]\nhost_library: /lib/a.so\n\
                 [mcu b]\nhost_library: /lib/b.so\nuuid: {taken}\n",
            ),
            "mcu a",
        )
        .unwrap();
        let a = libraries.identity("/lib/a.so").unwrap();
        let b = libraries.identity("/lib/b.so").unwrap();
        assert_ne!(a, b, "two paths are two devices");
        assert_ne!(a, natural, "its natural candidate was claimed");
        assert_ne!(b, natural);
    }

    /// The uuid an attach names is the section's own when it wrote one, and the
    /// path's identity when it did not.
    #[test]
    fn named_uuid_prefers_the_sections_own_uuid_then_the_identity() {
        let libraries = classify_from(
            &format!(
                "[mcu]\nhost_library: {LIB}\nuuid: 11aa22bb33cc\n\
                 [mcu tool]\nhost_library: {LIB}\n"
            ),
            "mcu",
        )
        .unwrap();
        let declared = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        assert_eq!(libraries.named_uuid(LIB, Some(declared)), Some(declared));
        assert_eq!(libraries.named_uuid(LIB, None), libraries.identity(LIB));
        assert_eq!(libraries.named_uuid("/lib/none.so", None), None);
    }
}
