//! Which `serial:` paths are shared, and whether the MCUs on one agree on baud.
//!
//! A serial port normally belongs to a single `[mcu]`: one path, one device, one
//! rate. This module is the load-time pass that tells the two shapes apart. It
//! reads every `[mcu]` — the bare section and the `[mcu <name>]` prefixes — that
//! names a `serial:` path and groups them **by path string**: a path one section
//! names is exclusive (the existing single-MCU mode), a path two or more name is
//! *shared*, the mode several MCUs on one port use. Nothing here opens a port or
//! touches a transport; it only classifies and validates, so the answer is ready
//! before anything connects. Mapping a shared group onto the transport is the
//! next layer's work.
//!
//! The one check is baud agreement: MCUs on one path talk at one rate, so two
//! different `baud` values for the same path is a config error that names both
//! MCUs, the path, and the two rates. An absent `baud` means [`DEFAULT_BAUD`] —
//! the same default the section's own transport parse applies
//! (`config/mcu.rs`) — so a shared path where one MCU writes no rate and another
//! writes [`DEFAULT_BAUD`] is consistent, not a conflict.
//!
//! Like [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit), this
//! is a pre-scan: it reads a sibling section's raw option text rather than going
//! through the tracked getters, so it never records a read on the shared access
//! tracker and thus never exempts a `[mcu]` option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account. The section's own factory is the one that reads and validates
//! `serial`/`baud`; a rate that does not parse is left for it to report, and is
//! skipped for the comparison here exactly as an unparsable `canbus_nodeid` is
//! left unreserved.

use std::collections::BTreeMap;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::interface::devices::serial::DEFAULT_BAUD;

/// The serial paths the config names, grouped by path.
///
/// Built once at load by [`SerialPorts::classify`]; the transport layer asks it
/// whether a path is shared and who is on it before opening the port.
#[derive(Debug, Default)]
pub struct SerialPorts {
    /// path → the MCU names naming it, in `[mcu]` declaration order: the bare
    /// `[mcu]` first, then every `[mcu <name>]` in file order, the order
    /// [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit) reads
    /// them in. A `BTreeMap` so a conflicting path is reported in the same order
    /// on every run.
    by_path: BTreeMap<String, Vec<String>>,
}

impl SerialPorts {
    /// Group every `[mcu]` section's `serial:` path and check the bauds agree.
    ///
    /// A section without a `serial` option (host library, CAN, the `test:`
    /// simulator) is not on a serial path and is skipped. The section list is
    /// the same two families
    /// [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit) scans:
    /// the bare `[mcu]` and every `[mcu <name>]`.
    ///
    /// # Errors
    /// A config error when one path carries two different bauds. Two sections
    /// naming a path with no rate at all agree, since both mean
    /// [`DEFAULT_BAUD`].
    pub fn classify(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        // path → (mcu name, the baud it declares). A `None` baud means the
        // section names one that does not parse: its own factory reports that,
        // so it is skipped for the comparison instead of becoming a second
        // error here.
        let mut groups: BTreeMap<String, Vec<(String, Option<u32>)>> = BTreeMap::new();
        for section in config
            .sibling("mcu")
            .into_iter()
            .chain(config.sibling_prefix_sections("mcu"))
        {
            let Some(path) = section.section().get_text("serial") else {
                continue;
            };
            groups
                .entry(path)
                .or_default()
                .push((section.identifier(), declared_baud(&section)));
        }

        let mut by_path: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for (path, members) in groups {
            check_baud_agrees(&path, &members)?;
            by_path.insert(path, members.into_iter().map(|(name, _)| name).collect());
        }
        Ok(Self { by_path })
    }

    /// Whether two or more MCUs name `path`. A path no MCU names is not shared.
    pub fn is_shared(&self, path: &str) -> bool {
        self.peers(path).len() >= 2
    }

    /// The MCUs that name `path`, in declaration order — one for an exclusive
    /// port, two or more for a shared one. Empty when no `[mcu]` names it.
    pub fn peers(&self, path: &str) -> &[String] {
        self.by_path.get(path).map(Vec::as_slice).unwrap_or(&[])
    }
}

/// The baud a `[mcu]` section declares: its `baud` option, or [`DEFAULT_BAUD`]
/// when it writes none. `None` when it writes one that is not a positive
/// integer — the section's own transport parse rejects that value
/// (`config/mcu.rs`), so there is nothing to compare here.
fn declared_baud(section: &ConfigWrapper) -> Option<u32> {
    let Some(text) = section.section().get_text("baud") else {
        return Some(DEFAULT_BAUD);
    };
    match text.trim().parse::<u32>() {
        Ok(baud) if baud > 0 => Some(baud),
        _ => None,
    }
}

/// Reject a path whose sections name two different bauds, naming both MCUs.
///
/// Sections whose rate does not parse are skipped: their own factory reports
/// them. The first parsed rate is the reference every later one must match, so
/// the error names the two sections whose rates disagree, not just the path.
fn check_baud_agrees(path: &str, members: &[(String, Option<u32>)]) -> Result<(), ConfigError> {
    let mut first: Option<(&str, u32)> = None;
    for (name, baud) in members {
        let Some(baud) = baud else {
            continue;
        };
        match first {
            None => first = Some((name.as_str(), *baud)),
            Some((first_name, first_baud)) if first_baud != *baud => {
                return Err(ConfigError::new(format!(
                    "shared serial port '{path}': MCU '{name}' sets baud {baud} but MCU \
                     '{first_name}' sets baud {first_baud}; a serial port shared by \
                     several MCUs needs one baud"
                )));
            }
            Some(_) => {}
        }
    }
    Ok(())
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
    fn classify(text: &str) -> Result<SerialPorts, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        SerialPorts::classify(&wrapper)
    }

    /// A path one MCU names is exclusive; `peers` reports that one MCU.
    #[test]
    fn a_single_mcu_on_a_path_is_exclusive() {
        let ports = classify("[mcu]\nserial: /dev/ttyACM0\n").expect("one MCU names the path");
        assert!(!ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu"]);

        // A path no MCU names is not shared, and has no peers.
        assert!(!ports.is_shared("/dev/ttyACM1"));
        assert!(ports.peers("/dev/ttyACM1").is_empty());
    }

    /// Two `[mcu <name>]` sections on one path are a shared group, both named.
    #[test]
    fn two_prefixed_mcus_on_one_path_are_shared() {
        let ports = classify(
            "[mcu a]\nserial: /dev/ttyACM0\n\
             [mcu b]\nserial: /dev/ttyACM0\n",
        )
        .expect("the two MCUs agree on baud");
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu a", "mcu b"]);
    }

    /// The bare `[mcu]` and a `[mcu <name>]` form one shared group: both families
    /// are scanned.
    #[test]
    fn the_bare_section_and_a_prefixed_one_share_a_path() {
        let ports = classify(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
        )
        .expect("the two MCUs agree on baud");
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu", "mcu tool"]);
    }

    /// Two different bauds on one path are rejected, and the error names both
    /// MCUs, the path, and both rates.
    #[test]
    fn a_shared_path_with_two_bauds_is_rejected() {
        let err = classify(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: 250000\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 115200\n",
        )
        .unwrap_err();
        let text = err.to_string();
        assert!(text.contains("mcu a"), "{text}");
        assert!(text.contains("mcu b"), "{text}");
        assert!(text.contains("/dev/ttyACM0"), "{text}");
        assert!(text.contains("250000"), "{text}");
        assert!(text.contains("115200"), "{text}");
    }

    /// A missing `baud` means [`DEFAULT_BAUD`], so an MCU that writes no rate and
    /// one that writes the default agree.
    #[test]
    fn an_absent_baud_means_the_default_rate() {
        let ports = classify(
            "[mcu a]\nserial: /dev/ttyACM0\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 250000\n",
        )
        .expect("the absent rate is the default, so both agree");
        assert!(ports.is_shared("/dev/ttyACM0"));

        // One MCU on its own with no rate is exclusive, and its path is still
        // known.
        let ports = classify("[mcu]\nserial: /dev/ttyACM0\n").expect("one MCU");
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu"]);
    }

    /// A rate that does not parse is not reported here: the section's own
    /// factory owns that error, so it is skipped for the comparison rather than
    /// turned into a second one.
    #[test]
    fn an_unparsable_baud_is_left_to_the_sections_own_factory() {
        let ports = classify(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: fast\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 115200\n",
        )
        .expect("the unparsable rate is skipped, not turned into a baud conflict here");
        assert!(ports.is_shared("/dev/ttyACM0"));
    }

    /// A section without `serial:` is ignored: a `host_library` MCU has a path,
    /// but it is not a serial path.
    #[test]
    fn sections_without_serial_are_ignored() {
        let ports = classify(
            "[mcu]\nhost_library: /nonexistent/libklipper_host.so\n\
             [mcu tool]\nhost_library: /nonexistent/libklipper_host.so\n",
        )
        .expect("no serial path to check");
        assert!(!ports.is_shared("/nonexistent/libklipper_host.so"));
        assert!(ports.peers("/nonexistent/libklipper_host.so").is_empty());
    }

    /// The scan records no reads: it must not exempt a sibling `[mcu]` section's
    /// options from `check_unused` on its own account — that is the section's own
    /// factory's job.
    #[test]
    fn classify_records_no_reads() {
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [mcu tool]\nserial: /dev/ttyACM0\nbaud: 250000\n",
        )
        .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let access = AccessTracking::shared();
        let wrapper =
            ConfigWrapper::with_config(section, std::sync::Arc::clone(&access), None, &config);

        SerialPorts::classify(&wrapper).expect("the two MCUs agree on baud");

        assert!(
            access.sections().is_empty(),
            "classify read {:?}, which would exempt them from check_unused",
            access.sections()
        );
    }
}
