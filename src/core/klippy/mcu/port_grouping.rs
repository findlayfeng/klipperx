//! Which `[mcu]` sections sit on each `[uart_device]` / `[can_device]` port,
//! and the conflicts that make the topology impossible.
//!
//! A downstream port (M1's `[uart_device <name>]` / `[can_device <name>]`,
//! parsed by [`bridge_ports`](super::bridge_ports)) is defined once by its own
//! section; the `[mcu]` sections whose devices hang on it reference it by name
//! (`uart:` / `can:`). This module is the load-time pass that reads both sides
//! and groups the referencing sections **by port section** — the shape
//! [`serial_ports`](super::serial_ports) gives a `serial:` path and
//! [`host_libraries`](super::host_libraries) gives a `host_library:` path —
//! so the rules that depend on how many boards share a wire can be checked in
//! one place, and the attach layer can ask who is on a port before anything
//! connects.
//!
//! The rules, per port (plan §3):
//!
//! - **exactly one** referencing `[mcu]`, uart port, no `uuid` — **legacy**
//!   (rule 5): a device with no admin channel, spoken to in raw frames;
//! - exactly one, with a `uuid` — exclusive but named: the device answers the
//!   mux itself;
//! - **two or more** (rule 4) — shared, and every one of them must write a
//!   **distinct** `uuid` so a mux header names one board, never two;
//! - **any** member of a **CAN** port must write a `uuid` (rule 6): CAN
//!   addressing is by uuid, so a CAN port is never legacy.
//!
//! A `uuid`/`canbus_uuid` is compared the way `mcu/link_grouping.rs`
//! spells it for both link passes (`host_library:` paths and ports):
//! trimmed and lowercased, so two spellings of one identity are one identity;
//! whether a uuid parses is the `[mcu]` section's own factory's call, and an
//! unparsable one still takes part in the duplicate check so two spellings of
//! the same bad value are still one board claimed twice. The grouping itself
//! and the uuid-distinguishability judgment are that module's shared core —
//! one judgment for both link kinds; the wording and the shared/legacy
//! verdict stay here.
//!
//! Two conflicts are config errors naming the sections involved: a shared port
//! (or a CAN port) with a member that writes no `uuid`, and two members writing
//! the same one. "A uuid-less member on a shared port" and "legacy on a shared
//! port" are the **same condition** under rule 5's definition — a legacy
//! device may only sit alone — so it is one check with one message, not two.
//!
//! One more error lives on this pass: the `[mcu]` a port section names as its
//! bridge (`*_mcu`) may not itself reference a port. A bridge is the upstream
//! side of the link; a section that is both the bridge of a port and a device
//! on one (its own or another's) contradicts the topology rule 1 derives from
//! the references. Whether the `*_mcu` value names an existing `[mcu]` section
//! at all is **not** checked here — that is the port section's own factory's
//! error (`bridge_ports::require_mcu`), so one bad config is reported once, in
//! one place.
//!
//! What this pass deliberately leaves to others: a `uart:`/`can:` reference
//! naming no port section is the `[mcu]` transport parse's error
//! (`config/mcu.rs`), and every check that needs the firmware's identify
//! answer (port exists on the bridge, type matches the oid, single-device
//! flag) is the connect pass's (plan §4.3).
//!
//! Like the two sibling passes, this is a pre-scan: it reads raw option text
//! (`section().get_text`, never a tracked getter), so it never records a read
//! on the shared access tracker and thus never exempts a `[mcu]` option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account. The sections' own factories are the ones that read and validate
//! their options.
//!
//! Every query is keyed by the port **section identifier** — `uart_device
//! uart0`, the spelling [`ConfigSection::identifier`]
//! (crate::core::klippy::config::section) builds and the loader registers a
//! section under — which keeps a uart and a can port that happen to share a
//! sub (`[uart_device foo]` + `[can_device foo]`) apart, and is the spelling a
//! transport parse that just read `uart: foo` can build directly.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::mcu::link_grouping::{
    declared_uuid, distinguishability, group_by_key, quoted, Distinguishability, Member,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The registry key this pass is looked up and registered under.
pub const PORT_GROUPS_OBJECT: &str = "port_groups";

/// What a port section is declared as, which decides its rules.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PortKind {
    /// `[uart_device <name>]`: may host a legacy device when it is alone.
    Uart,
    /// `[can_device <name>]`: every member must write a `uuid` (rule 6).
    Can,
}

impl PortKind {
    /// The section type a port of this kind is declared as: `uart_device` or
    /// `can_device`.
    ///
    /// A caller that holds a kind and a sub (a transport parse that just read
    /// `uart: <sub>`) builds the query key with this:
    /// `format!("{} {sub}", kind.section_type())`.
    pub fn section_type(self) -> &'static str {
        match self {
            PortKind::Uart => "uart_device",
            PortKind::Can => "can_device",
        }
    }

    /// The `[mcu]` option that references a port of this kind: `uart` or `can`.
    fn reference_option(self) -> &'static str {
        match self {
            PortKind::Uart => "uart",
            PortKind::Can => "can",
        }
    }
}

/// The `[mcu]` sections on each port, and the verdicts that follow.
///
/// Built once at load by [`PortGroups::classify`]; the transport and attach
/// layers ask it who is on a port, whether the port is shared, and whether its
/// device is legacy before opening anything.
#[derive(Debug, Default)]
pub struct PortGroups {
    /// port section identifier → the port. A `BTreeMap` so a conflict is
    /// reported in the same order on every run.
    by_identifier: BTreeMap<String, PortGroup>,
}

/// One port section and the `[mcu]` sections that reference it.
#[derive(Debug)]
struct PortGroup {
    /// What the port section is declared as.
    kind: PortKind,
    /// The port section's `*_mcu` value: the `[mcu]` the port's hardware sits
    /// on (the bridge), `mcu` for the bare section. `None` only for a section
    /// missing the option — its own factory refuses that.
    bridge: Option<String>,
    /// The referencing MCUs' section identifiers, in `[mcu]` declaration
    /// order: the bare `[mcu]` first, then every `[mcu <name>]` in file order,
    /// the order [`McuIds::reserve_explicit`](super::ids::reserve_explicit)
    /// reads them in.
    members: Vec<String>,
    /// Each member's `uuid` (or `canbus_uuid`), trimmed and lowercased —
    /// the shared `mcu/link_grouping.rs` spelling both link passes compare
    /// by, so two spellings of one identity are one identity. `None` when
    /// it writes neither. Section identifiers are unique, so the map is
    /// one entry per member.
    uuids: BTreeMap<String, Option<String>>,
}

impl PortGroups {
    /// Group every `[mcu]` section's `uart:`/`can:` reference by the port
    /// section it names, and reject the topologies that cannot stand.
    ///
    /// The port side is every `[uart_device <name>]` / `[can_device <name>]`
    /// section; the MCU side is the same two families
    /// [`McuIds::reserve_explicit`](super::ids::reserve_explicit) scans: the
    /// bare `[mcu]` and every `[mcu <name>]`. A reference naming no port
    /// section is not grouped and not reported here — resolving it is the
    /// transport parse's job (`config/mcu.rs`).
    ///
    /// # Errors
    /// A port whose topology cannot stand — a uuid-less member on a shared or
    /// CAN port, two members writing the same `uuid`, or a bridge MCU that is
    /// itself a downstream device. The error names the sections involved.
    pub fn classify(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        // The port sections: identifier → what they declare. The `*_mcu` value
        // is read raw; whether it names an existing `[mcu]` is the port
        // factory's check (module docs), so this pass sees the text and no
        // error.
        let mut ports: BTreeMap<String, PortSection> = BTreeMap::new();
        for (prefix, kind, mcu_option) in [
            ("uart_device", PortKind::Uart, "uart_mcu"),
            ("can_device", PortKind::Can, "can_mcu"),
        ] {
            for section in config.sibling_prefix_sections(prefix) {
                ports.insert(
                    section.identifier(),
                    PortSection {
                        kind,
                        bridge: section.section().get_text(mcu_option),
                    },
                );
            }
        }

        // Which port each bridge name comes from, for the bridge-is-also-a-
        // device refusal. First in identifier order wins, so the message is
        // deterministic however many ports one MCU bridges.
        let mut bridge_owner: BTreeMap<String, String> = BTreeMap::new();
        for (identifier, port) in &ports {
            if let Some(bridge) = &port.bridge {
                bridge_owner
                    .entry(bridge.clone())
                    .or_insert_with(|| identifier.clone());
            }
        }

        // The referencing `[mcu]` sections' claims: the port section
        // identifier they name (`uart_device uart0`, the spelling `ports` is
        // keyed by) and the uuid they write. A section that writes both `uart:`
        // and `can:` is a conflict its own transport parse reports; here it is
        // one device, on the first reference written.
        let mut claims: Vec<(String, Member)> = Vec::new();
        for section in config
            .sibling("mcu")
            .into_iter()
            .chain(config.sibling_prefix_sections("mcu"))
        {
            let identifier = section.identifier();
            // The `*_mcu` spelling of this section: `mcu` for the bare one,
            // the sub otherwise.
            let name = section.section().sub.as_deref().unwrap_or("mcu");
            if let Some(owner) = bridge_owner.get(name) {
                if let Some((kind, referenced)) = port_reference(&section) {
                    return Err(ConfigError::new(format!(
                        "MCU '{identifier}' is the bridge of port '{owner}', but it is also a \
                         downstream device: it references port '{referenced}' via '{}'",
                        kind.reference_option()
                    )));
                }
            }
            if let Some((kind, referenced)) = port_reference(&section) {
                claims.push((
                    format!("{} {referenced}", kind.section_type()),
                    Member {
                        identifier,
                        uuid: declared_uuid(&section),
                    },
                ));
            }
        }

        let mut members = group_by_key(claims);
        let mut by_identifier: BTreeMap<String, PortGroup> = BTreeMap::new();
        for (identifier, port) in ports {
            let group_members = members.remove(&identifier).unwrap_or_default();
            validate_members(&identifier, port.kind, &group_members)?;
            by_identifier.insert(
                identifier,
                PortGroup {
                    kind: port.kind,
                    bridge: port.bridge,
                    members: group_members
                        .iter()
                        .map(|member| member.identifier.clone())
                        .collect(),
                    uuids: group_members
                        .iter()
                        .map(|member| (member.identifier.clone(), member.uuid.clone()))
                        .collect(),
                },
            );
        }
        Ok(Self { by_identifier })
    }

    /// The MCUs that reference `port`, in declaration order — one for an
    /// exclusive port, two or more for a shared one. Empty when no `[mcu]`
    /// names it.
    pub fn peers(&self, port: &str) -> &[String] {
        self.by_identifier
            .get(port)
            .map(|group| group.members.as_slice())
            .unwrap_or(&[])
    }

    /// Whether two or more `[mcu]` sections reference `port`. A port no MCU
    /// references is not shared.
    pub fn is_shared(&self, port: &str) -> bool {
        self.peers(port).len() >= 2
    }

    /// Whether `port`'s one device is legacy (rule 5): exactly one referencing
    /// `[mcu]`, on a uart port, writing no `uuid` — so the port is spoken to
    /// in raw frames, with no mux header. Never true for a shared port (its
    /// members all write uuids, checked at classify) nor for a CAN port
    /// (rule 6).
    pub fn is_legacy(&self, port: &str) -> bool {
        self.by_identifier.get(port).is_some_and(|group| {
            group.kind == PortKind::Uart
                && group.members.len() == 1
                && group
                    .uuids
                    .get(&group.members[0])
                    .is_some_and(Option::is_none)
        })
    }

    /// The `[mcu]` the port section names as its bridge (`*_mcu`): `mcu` for
    /// the bare section, the sub otherwise. `None` when no port section `port`
    /// is (or it is missing the option — its own factory refuses that).
    pub fn bridge(&self, port: &str) -> Option<&str> {
        self.by_identifier
            .get(port)
            .and_then(|group| group.bridge.as_deref())
    }

    /// What `port` is declared as. `None` when no port section `port` is.
    pub fn kind(&self, port: &str) -> Option<PortKind> {
        self.by_identifier.get(port).map(|group| group.kind)
    }
}

impl PrinterObject for PortGroups {
    /// The registry reports no status, exactly as the id allocator, the serial
    /// grouping and the host-library grouping define none.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible: nothing here is a status a client can query.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// Reject a port whose topology cannot stand.
///
/// The transport parse (`config/mcu.rs`) calls this as it parses a `uart:` /
/// `can:` section, so a conflict is a config error and the config fails to
/// load — the same place [`host_libraries::validate`](super::host_libraries)
/// rejects a path whose MCUs cannot be told apart. It runs the same grouping
/// [`PortGroups::classify`] does and keeps only the verdict.
///
/// # Errors
/// As [`PortGroups::classify`].
pub fn validate(config: &ConfigWrapper) -> Result<(), ConfigError> {
    PortGroups::classify(config).map(|_| ())
}

/// The single grouping, created on first demand.
///
/// The first `[mcu]` factory to load builds it from the whole config and
/// registers it; every later one finds it already there and **does not**
/// re-classify, so the grouping is a load-time constant rather than something
/// each section recomputes. Mirrors [`serial_ports::ensure`](super::serial_ports)
/// and [`host_libraries::ensure`](super::host_libraries).
///
/// # Errors
/// A topology conflict ([`PortGroups::classify`]), or a registry key that is
/// already taken by a different object.
pub fn ensure(
    printer: &Arc<Printer>,
    config: &ConfigWrapper,
) -> Result<Arc<PortGroups>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<PortGroups>(PORT_GROUPS_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(PortGroups::classify(config)?);
    printer.add_object(PORT_GROUPS_OBJECT, object.clone())?;
    Ok(object)
}

/// One port section as this pass sees it: what it declares and the bridge it
/// names. Its identifier — `{type} {sub}` — is the map key and the name
/// references are written with.
struct PortSection {
    kind: PortKind,
    bridge: Option<String>,
}

/// The port a `[mcu]` section references, if it references one: the kind its
/// option spells and the port's sub, trimmed. The first reference written wins
/// when it writes both options — the double reference itself is the transport
/// parse's conflict to report.
///
/// Read raw, like every scan here, so it records no access.
///
/// The `uuid` itself is read by the shared `mcu/link_grouping.rs` rule
/// ([`declared_uuid`]) both link passes compare by.
fn port_reference(section: &ConfigWrapper) -> Option<(PortKind, String)> {
    for kind in [PortKind::Uart, PortKind::Can] {
        if let Some(text) = section.section().get_text(kind.reference_option()) {
            let sub = text.trim();
            if !sub.is_empty() {
                return Some((kind, sub.to_string()));
            }
        }
    }
    None
}

/// Reject a port whose members cannot be told apart or address their boards.
///
/// Members are the grouped members [`PortGroups::classify`] gathered for the
/// port, in declaration order. The judgment is the shared distinguishability
/// rule (`mcu/link_grouping.rs`); what differs per port is how many members
/// may write no `uuid` and how the refusal reads:
///
/// - CAN (rule 6): every member must write one — CAN addressing is by uuid,
///   so a uuid-less member there can never talk, legacy or not.
/// - Shared uart (rule 4): every member must write one too — this is also the
///   legacy refusal, since rule 5 defines legacy as a single-device-port
///   shape; one condition, one message.
/// - Alone on a uart port one member may omit it (rule 5's legacy shape), and
///   two members writing one identity name the same board twice on any port.
fn validate_members(
    identifier: &str,
    kind: PortKind,
    members: &[Member],
) -> Result<(), ConfigError> {
    let named_by_rule = kind == PortKind::Can || members.len() >= 2;
    let Some(violation) = distinguishability(members, usize::from(!named_by_rule)) else {
        return Ok(());
    };
    let names = |identifiers: &[String]| {
        quoted(&identifiers.iter().map(String::as_str).collect::<Vec<_>>())
    };
    match violation {
        Distinguishability::MissingUuid(without) => {
            let why = if kind == PortKind::Can {
                "a CAN port addresses its boards by uuid, so every MCU on it must write one"
            } else {
                "a port two or more MCUs reference is shared, and every MCU on it must write a \
                 uuid to be told apart"
            };
            Err(ConfigError::new(format!(
                "port '{identifier}': {why}, but these name it without one: {}",
                names(&without)
            )))
        }
        Distinguishability::DuplicateUuid { uuid, members } => Err(ConfigError::new(format!(
            "port '{identifier}': {} set the same uuid {uuid}, so the boards cannot be \
             told apart",
            names(&members)
        ))),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// Parse `text` and group through its first `[mcu]` section.
    ///
    /// Any `[mcu]`-family wrapper reaches the same sibling sections, so which
    /// one is wrapped does not change the answer.
    fn classify(text: &str) -> Result<PortGroups, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        PortGroups::classify(&wrapper)
    }

    /// One MCU on a uart port is exclusive and, with a uuid, named — not
    /// legacy. The port reports its kind and bridge, and a port no MCU
    /// references has no peers and is not legacy.
    #[test]
    fn one_mcu_on_a_uart_port_is_exclusive_and_named() {
        let groups = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [uart_device lonely]\nuart_mcu: bridge\nuart_oid: uart1\n",
        )
        .unwrap();

        assert!(!groups.is_shared("uart_device uart0"));
        assert_eq!(groups.peers("uart_device uart0"), ["mcu child"]);
        assert!(!groups.is_legacy("uart_device uart0"));
        assert_eq!(groups.bridge("uart_device uart0"), Some("bridge"));
        assert_eq!(groups.kind("uart_device uart0"), Some(PortKind::Uart));

        // A port no `[mcu]` references still reports what its section declares,
        // and is not legacy: legacy is a device shape, not an absence of one.
        assert!(groups.peers("uart_device lonely").is_empty());
        assert!(!groups.is_shared("uart_device lonely"));
        assert!(!groups.is_legacy("uart_device lonely"));
        assert_eq!(groups.bridge("uart_device lonely"), Some("bridge"));
    }

    /// The bare `[mcu]` and a `[mcu <name>]` with distinct uuids are a shared
    /// group, in declaration order — and neither is legacy.
    #[test]
    fn two_mcus_on_one_port_are_shared_and_never_legacy() {
        let groups = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu tool]\nuart: uart0\ncanbus_uuid: 22BB33CC44DD\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .unwrap();

        assert!(groups.is_shared("uart_device uart0"));
        assert_eq!(groups.peers("uart_device uart0"), ["mcu", "mcu tool"]);
        assert!(!groups.is_legacy("uart_device uart0"));
    }

    /// Exactly one referencing MCU, a uart port, no `uuid` — rule 5's legacy
    /// shape: raw frames, no mux header. The `canbus_uuid` alias counts as a
    /// uuid, so writing it takes the port out of legacy.
    #[test]
    fn a_single_uuidless_member_on_a_uart_port_is_legacy() {
        let groups = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [mcu named]\nuart: named0\nuuid: 11aa22bb33cc\n\
             [uart_device named0]\nuart_mcu: bridge\nuart_oid: uart1\n\
             [mcu aliased]\nuart: aliased0\ncanbus_uuid: 22bb33cc44dd\n\
             [uart_device aliased0]\nuart_mcu: bridge\nuart_oid: uart2\n",
        )
        .unwrap();

        assert!(groups.is_legacy("uart_device uart0"));
        assert!(!groups.is_legacy("uart_device named0"));
        assert!(!groups.is_legacy("uart_device aliased0"));
        // A port the config does not declare is not legacy either.
        assert!(!groups.is_legacy("uart_device nowhere"));
    }

    /// A CAN port's member is never legacy: rule 6 gives it a uuid or refuses
    /// the config, and the port reports its kind and bridge.
    #[test]
    fn a_can_port_is_never_legacy() {
        let groups = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\nuuid: 11aa22bb33cc\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
        )
        .unwrap();

        assert_eq!(groups.kind("can_device can0"), Some(PortKind::Can));
        assert_eq!(groups.bridge("can_device can0"), Some("bridge"));
        assert!(!groups.is_shared("can_device can0"));
        assert!(!groups.is_legacy("can_device can0"));
    }

    /// A shared port with a member that writes no `uuid` cannot tell its
    /// boards apart: one error naming the port and the uuid-less member. This
    /// is also the "legacy on a shared port" refusal — one condition.
    #[test]
    fn a_shared_port_with_a_uuidless_member_is_refused() {
        let err = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu tool]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .unwrap_err();

        assert!(err.to_string().contains("uart_device uart0"), "{err}");
        assert!(err.to_string().contains("'mcu tool'"), "{err}");
        assert!(err.to_string().contains("uuid"), "{err}");
    }

    /// Two members writing one identity name the same board twice: an error
    /// naming both sections and the uuid. The spellings differ in case and in
    /// alias, and the identity is compared the way `host_libraries` compares
    /// it — trimmed, lowercased, alias included.
    #[test]
    fn two_members_with_the_same_uuid_are_refused() {
        let err = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu a]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu b]\nuart: uart0\ncanbus_uuid: 11AA22BB33CC\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .unwrap_err();

        assert!(err.to_string().contains("uart_device uart0"), "{err}");
        assert!(err.to_string().contains("'mcu a'"), "{err}");
        assert!(err.to_string().contains("'mcu b'"), "{err}");
        assert!(err.to_string().contains("11aa22bb33cc"), "{err}");
    }

    /// A CAN port's member without a `uuid` cannot be addressed at all
    /// (rule 6): an error naming the port and the member.
    #[test]
    fn a_can_port_member_without_a_uuid_is_refused() {
        let err = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
        )
        .unwrap_err();

        assert!(err.to_string().contains("can_device can0"), "{err}");
        assert!(err.to_string().contains("'mcu child'"), "{err}");
        assert!(err.to_string().contains("uuid"), "{err}");
    }

    /// A `*_mcu` naming no `[mcu]` section is the port factory's error to
    /// report (`bridge_ports::require_mcu`), not a second one here: this pass
    /// keeps the written name so `bridge` can report it, and does not refuse
    /// the config.
    #[test]
    fn an_unknown_bridge_mcu_is_the_port_factories_error() {
        let groups = classify(
            "[mcu]\ntest: dict=/nonexistent\n\
             [uart_device uart0]\nuart_mcu: zboard\nuart_oid: uart0\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n",
        )
        .unwrap();

        assert_eq!(groups.bridge("uart_device uart0"), Some("zboard"));
        assert_eq!(groups.peers("uart_device uart0"), ["mcu child"]);
    }

    /// A reference naming no port section is not grouped and not this pass's
    /// error: resolving it belongs to the `[mcu]` transport parse.
    #[test]
    fn a_reference_to_no_port_section_is_left_to_the_transport_parse() {
        let groups = classify(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: nowhere\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .unwrap();

        assert!(groups.peers("uart_device uart0").is_empty());
        assert!(!groups.is_shared("uart_device nowhere"));
        assert_eq!(groups.kind("uart_device nowhere"), None);
    }

    /// The MCU a port names as its bridge may not be a downstream device on
    /// any port — rule 1's bridge shape. The error names the bridge section,
    /// the port it bridges, and the port it also references.
    #[test]
    fn a_bridge_mcu_may_not_reference_a_port() {
        let err = classify(
            "[mcu bridge]\nuart: uart1\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [uart_device uart1]\nuart_mcu: bridge\nuart_oid: uart1\n",
        )
        .unwrap_err();

        assert!(err.to_string().contains("'mcu bridge'"), "{err}");
        assert!(err.to_string().contains("bridge of port"), "{err}");
        assert!(err.to_string().contains("uart1"), "{err}");
    }

    /// `validate` is the same verdict the transport parse calls: a topology
    /// conflict is an error, a sound one is not.
    #[test]
    fn validate_reports_the_same_conflict() {
        let sound = "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n";
        let (config, _) = Config::from_text(sound).expect("the test config parses");
        let section = config
            .get_section("mcu bridge")
            .expect("the bridge section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        validate(&wrapper).expect("a sound topology validates");

        let conflicting = "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\n\
             [mcu tool]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n";
        let (config, _) = Config::from_text(conflicting).expect("the test config parses");
        let section = config
            .get_section("mcu bridge")
            .expect("the bridge section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        assert!(validate(&wrapper).is_err());
    }

    /// `ensure` registers one grouping under the registry key and hands every
    /// later caller the same object; it is not client-visible.
    #[test]
    fn ensure_registers_the_grouping_once() {
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let groups = ensure(&printer, &wrapper).expect("the grouping registers");
        let again = ensure(&printer, &wrapper).expect("the second call finds it");
        assert!(Arc::ptr_eq(&groups, &again), "one grouping, one load");
        assert_eq!(groups.peers("uart_device uart0"), ["mcu"]);
        assert!(!groups.is_queryable());
    }

    /// The scan records no reads: it must not exempt a sibling section's
    /// options from `check_unused` on its own account — that is each section's
    /// own factory's job.
    #[test]
    fn classify_records_no_reads() {
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu tool]\nuart: uart0\nuuid: 22bb33cc44dd\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        PortGroups::classify(&wrapper).expect("distinct uuids agree");

        assert!(
            access.sections().is_empty(),
            "classify read {:?}, which would exempt them from check_unused",
            access.sections()
        );
    }

    /// A wrapper that cannot reach its siblings sees no ports: a test's small
    /// wrapper, or a part reading a stored section back.
    #[test]
    fn a_wrapper_without_the_config_sees_no_ports() {
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::untracked(section);

        let groups = PortGroups::classify(&wrapper).expect("nothing to conflict");
        assert!(groups.peers("uart_device uart0").is_empty());
        assert_eq!(groups.kind("uart_device uart0"), None);
        assert_eq!(groups.bridge("uart_device uart0"), None);
        assert!(!groups.is_legacy("uart_device uart0"));
    }
}
