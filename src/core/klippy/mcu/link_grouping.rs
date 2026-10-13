//! The grouping core both `[mcu]` link passes share.
//!
//! Two passes group `[mcu]` sections by the link they name —
//! [`host_libraries`](super::host_libraries) by the `host_library:` path and
//! [`port_grouping`](super::port_grouping) by the `[uart_device]` /
//! [`can_device`](super::port_grouping) section a `uart:`/`can:` reference
//! names — and both apply the one judgment a shared link lives or dies by:
//! can the boards on it be told apart? That judgment
//! ([`distinguishability`]) lives here, once, so the two cannot drift; the
//! grouping that feeds it ([`group_by_key`]) and the [`declared_uuid`]
//! spelling it compares by live beside it.
//!
//! What stays with each caller is its own wording and its own verdict: a
//! library path decides whether its frames carry the mux header
//! (`host_libraries::path_is_mux`), a port section whether it is shared or
//! legacy (`port_grouping::validate_members`), and each names its own
//! conflicts — a library path whose boards cannot be told apart, a port
//! whose members cannot.
//!
//! Like every scan this module reads raw option text
//! ([`ConfigWrapper::section`] + `get_text`, never a tracked getter), so a
//! grouping records no access and never exempts an option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account.

use std::collections::BTreeMap;

use crate::core::klippy::config::ConfigWrapper;

/// One `[mcu]` section on a link: who it is and the identity it writes.
#[derive(Debug)]
pub(crate) struct Member {
    /// The section's identifier — `mcu` for the bare one, `mcu <name>`
    /// otherwise — the spelling errors name it by.
    pub(crate) identifier: String,
    /// The `uuid`/`canbus_uuid` the section writes, trimmed and lowercased
    /// ([`declared_uuid`]), or `None` when it writes neither.
    pub(crate) uuid: Option<String>,
}

/// The `uuid` a `[mcu]` section writes for itself: the canonical `uuid` or
/// its `canbus_uuid` alias, trimmed and lowercased so two spellings of one
/// identity compare equal, or `None` when it writes neither.
///
/// Read raw through [`ConfigWrapper::section`], so the scan records no
/// access (see the module docs). Whether a uuid parses is the `[mcu]`
/// section's own factory's call; the grouping compares text only.
pub(crate) fn declared_uuid(section: &ConfigWrapper) -> Option<String> {
    section
        .section()
        .get_text("uuid")
        .or_else(|| section.section().get_text("canbus_uuid"))
        .map(|text| text.trim().to_ascii_lowercase())
}

/// Group link claims by their key: key order, members in claim order.
///
/// This is the one grouping both link passes do — their scans collect
/// `(link key, member)` pairs (a `host_library:` path, a port section
/// identifier like `uart_device uart0`) and feed them in. A `BTreeMap` so
/// every later step — conflict reporting, identity assignment — sees the
/// same order on every run.
pub(crate) fn group_by_key<I>(claims: I) -> BTreeMap<String, Vec<Member>>
where
    I: IntoIterator<Item = (String, Member)>,
{
    let mut groups: BTreeMap<String, Vec<Member>> = BTreeMap::new();
    for (key, member) in claims {
        groups.entry(key).or_default().push(member);
    }
    groups
}

/// The violation that leaves a link's members unroutable.
#[derive(Debug)]
pub(crate) enum Distinguishability {
    /// These members write no `uuid` and fewer were allowed to
    /// ([`distinguishability`]'s `allowed_uuidless`), in declaration order.
    MissingUuid(Vec<String>),
    /// This uuid is written by these members, in declaration order — one
    /// identity claimed more than once, so a header built from it would
    /// address every claimant at once.
    DuplicateUuid { uuid: String, members: Vec<String> },
}

/// Judge whether `members` can be told apart, given how many of them may
/// write no `uuid` (`allowed_uuidless`: 0 — every board must be named; 1 —
/// one may go unnamed, the link's own device or a legacy port's sole
/// device).
///
/// Returns the first violation — too many uuid-less members before a
/// duplicate — or `None` when the link is routable. The reported duplicate
/// is the lowest claimed uuid in `BTreeMap` order, so a config with several
/// duplicates is reported with the same pair on every run. Each caller
/// formats the verdict in its own words (`mcu/host_libraries.rs`,
/// `mcu/port_grouping.rs`); this function is the whole shared judgment.
pub(crate) fn distinguishability(
    members: &[Member],
    allowed_uuidless: usize,
) -> Option<Distinguishability> {
    let missing = members
        .iter()
        .filter(|member| member.uuid.is_none())
        .map(|member| member.identifier.clone())
        .collect::<Vec<_>>();
    if missing.len() > allowed_uuidless {
        return Some(Distinguishability::MissingUuid(missing));
    }
    let mut by_uuid: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for member in members {
        if let Some(uuid) = &member.uuid {
            by_uuid
                .entry(uuid.as_str())
                .or_default()
                .push(&member.identifier);
        }
    }
    by_uuid
        .iter()
        .find(|(_, names)| names.len() >= 2)
        .map(|(uuid, names)| Distinguishability::DuplicateUuid {
            uuid: (*uuid).to_string(),
            members: names.iter().map(|name| (*name).to_string()).collect(),
        })
}

/// The section names as a quoted, comma-separated list for an error message.
pub(crate) fn quoted(names: &[&str]) -> String {
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
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use std::sync::Arc;

    /// A member with an identifier and an optional uuid as written.
    fn member(identifier: &str, uuid: Option<&str>) -> Member {
        Member {
            identifier: identifier.to_string(),
            uuid: uuid.map(str::to_string),
        }
    }

    /// One uuid-less member is allowed by a rule of 1, refused by a rule of 0;
    /// too many are refused by either.
    #[test]
    fn the_missing_uuid_rule_counts_members() {
        let one = [member("mcu", None)];
        assert!(distinguishability(&one, 1).is_none());
        assert!(matches!(
            distinguishability(&one, 0),
            Some(Distinguishability::MissingUuid(names)) if names == ["mcu"]
        ));

        let two = [member("mcu a", None), member("mcu b", None)];
        assert!(matches!(
            distinguishability(&two, 1),
            Some(Distinguishability::MissingUuid(names)) if names == ["mcu a", "mcu b"]
        ));
    }

    /// The lowest duplicated uuid in key order is the one reported, with every
    /// member claiming it, in declaration order.
    #[test]
    fn a_duplicate_uuid_names_every_claimant() {
        // The spellings arrive already folded (`declared_uuid` trims and
        // lowercases both spellings before the grouping compares them).
        let members = [
            member("mcu a", Some("22bb33cc44dd")),
            member("mcu b", Some("11aa22bb33cc")),
            member("mcu c", Some("11aa22bb33cc")),
        ];
        let Some(Distinguishability::DuplicateUuid { uuid, members }) =
            distinguishability(&members, 0)
        else {
            panic!("a repeated uuid is a violation");
        };
        assert_eq!(uuid, "11aa22bb33cc");
        assert_eq!(members, ["mcu b", "mcu c"]);
    }

    /// A missing uuid is reported before a duplicate: it blocks every header,
    /// the duplicate only the one identity.
    #[test]
    fn missing_uuid_comes_before_a_duplicate() {
        let members = [
            member("mcu a", Some("11aa22bb33cc")),
            member("mcu b", None),
            member("mcu c", Some("11aa22bb33cc")),
        ];
        assert!(matches!(
            distinguishability(&members, 0),
            Some(Distinguishability::MissingUuid(_))
        ));
    }

    /// Distinct uuids — or a lone member each — are no violation at all.
    #[test]
    fn told_apart_members_pass() {
        let shared = [
            member("mcu a", Some("11aa22bb33cc")),
            member("mcu b", Some("22bb33cc44dd")),
        ];
        assert!(distinguishability(&shared, 0).is_none());
        let single = [member("mcu", Some("11aa22bb33cc"))];
        assert!(distinguishability(&single, 0).is_none());
        assert!(distinguishability(&[], 0).is_none());
    }

    /// The scan reads raw text: it records no access on the shared tracker.
    #[test]
    fn declared_uuid_records_no_reads() {
        let (config, _) = Config::from_text("[mcu]\nuuid: 11AA22BB33CC\n").expect("parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        assert_eq!(
            declared_uuid(&wrapper).as_deref(),
            Some("11aa22bb33cc"),
            "trimmed, lowercased, alias folded"
        );
        assert!(
            access.sections().is_empty(),
            "the scan read a tracked option"
        );
    }

    /// The alias is one spelling of the same identity, folded the same way;
    /// a section writing neither reports none.
    #[test]
    fn declared_uuid_folds_both_spellings() {
        let mut section = ConfigSection::new("mcu", None);
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single(" 11AA22BB33CC \n".to_string()),
        );
        let wrapper = ConfigWrapper::untracked(&section);
        assert_eq!(declared_uuid(&wrapper).as_deref(), Some("11aa22bb33cc"));

        let section = ConfigSection::new("mcu", None);
        let wrapper = ConfigWrapper::untracked(&section);
        assert_eq!(declared_uuid(&wrapper), None);
    }

    /// Claims group by key, keys in order, members in claim order.
    #[test]
    fn grouping_collects_claims_per_key() {
        let claims = vec![
            ("/lib/b.so".to_string(), member("mcu y", None)),
            (
                "/lib/a.so".to_string(),
                member("mcu x", Some("11aa22bb33cc")),
            ),
            ("/lib/a.so".to_string(), member("mcu z", None)),
        ];
        let groups = group_by_key(claims);
        let keys = groups.keys().map(String::as_str).collect::<Vec<_>>();
        assert_eq!(keys, ["/lib/a.so", "/lib/b.so"]);
        let a = &groups["/lib/a.so"];
        assert_eq!(a[0].identifier, "mcu x");
        assert_eq!(a[1].identifier, "mcu z");
    }
}
