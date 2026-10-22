//! Which `serial:` paths are shared, who is on one, and at what baud.
//!
//! A serial port normally belongs to a single `[mcu]`: one path, one device, one
//! rate. This module is the load-time pass that tells the two shapes apart. It
//! reads every `[mcu]` — the bare section and the `[mcu <name>]` prefixes — that
//! names a `serial:` path and groups them **by path string**: a path one section
//! names is exclusive (the existing single-MCU mode), a path two or more name is
//! *shared*, the mode several MCUs on one port use. Nothing here opens a port or
//! touches a transport; it only classifies and picks the rate, so the answer is
//! ready before anything connects. Mapping a shared group onto the transport is
//! the next layer's work.
//!
//! Being **addressed by id** is a slightly wider verdict than being shared: a
//! path a `[uart_device]`/`[can_device]` names as its bridge carries the bridge's
//! downstream devices' frames too, so a single `[mcu]` on it still needs the
//! per-id header ([`SerialPorts::is_id_addressed`]). The transport decision asks
//! that wider question; [`SerialPorts::is_shared`] keeps the narrower one (how
//! many `[mcu]` sections name the path).
//!
//! Rule 7 counts the downstream devices a bridge carries
//! ([`SerialPorts::downstream_devices`], from [`bridge_downstream_counts`]),
//! and that count is one reason a path is addressed by id. Being a bridge is a
//! **separate** reason that stays even at zero downstream devices: a muxed
//! bridge frames its output whatever is hanging on it, so decoding that output
//! as bare frames would misparse the per-id header. The zero-device case is
//! therefore a deliberate conservative reading, not an oversight.
//!
//! The one rule is baud, and the rate of a path is the **lowest** `baud` its
//! sections write — or [`DEFAULT_BAUD`] when none of them writes one. A section
//! that writes no `baud` has no vote of its own: it follows the rate the path
//! ends up with, so one MCU writing a rate and another writing nothing agree on
//! that rate instead of colliding with the default. Sections that write
//! different rates are worth saying out loud — they would drive the port at
//! different speeds — so the disagreement is `warn!`ed about, naming the path,
//! the rates, the MCUs that wrote them, and the rate chosen; the lowest wins and
//! the config still loads. Which rates are acceptable at all is the section's
//! own transport parse's call (`config/mcu.rs`), this pass only picks among the
//! rates it will accept.
//!
//! Like [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit), this
//! is a pre-scan: it reads a sibling section's raw option text rather than going
//! through the tracked getters, so it never records a read on the shared access
//! tracker and thus never exempts a `[mcu]` option from
//! [`check_unused`](crate::core::klippy::config::check_unused) on its own
//! account. The section's own factory is the one that reads and validates
//! `serial`/`baud`; a rate that does not parse is left for it to report, and is
//! skipped here exactly as an unparsable `canbus_nodeid` is left unreserved.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::interface::devices::serial::DEFAULT_BAUD;
use crate::core::klippy::mcu::bridge_ports::{bridge_downstream_counts, bridge_mcus};
use crate::core::klippy::printer::{Printer, PrinterObject};
use tracing::warn;

/// The registry key this pass is looked up and registered under.
pub const SERIAL_PORTS_OBJECT: &str = "serial_ports";

/// The serial paths the config names, grouped by path.
///
/// Built once at load by [`SerialPorts::classify`]; the transport layer asks it
/// whether a path is shared, who is on it, and at what rate before opening the
/// port.
#[derive(Debug, Default)]
pub struct SerialPorts {
    /// path → its `[mcu]` sections. A `BTreeMap` so a path whose MCUs write
    /// different rates is reported in the same order on every run.
    by_path: BTreeMap<String, PathGroup>,
}

/// One `serial:` path and the `[mcu]` sections that name it.
#[derive(Debug)]
struct PathGroup {
    /// The MCU names naming the path, in `[mcu]` declaration order: the bare
    /// `[mcu]` first, then every `[mcu <name>]` in file order, the order
    /// [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit) reads
    /// them in.
    members: Vec<String>,
    /// The rate they share, as [`SerialPorts::effective_baud`] reports it.
    baud: u32,
    /// Whether one of the path's MCUs is a bridge (a `[uart_device]` /
    /// `[can_device]` names it as its `*_mcu`). A bridge's link carries its
    /// downstream devices' frames, so the path is addressed by id even though
    /// one `[mcu]` names it — an id-addressing reason **besides** being shared
    /// ([`SerialPorts::is_id_addressed`]).
    bridge: bool,
    /// How many `[mcu]` destinations hang on this path's bridge(s): the sections
    /// that reference a `[uart_device]`/`[can_device]` port whose `*_mcu` is an
    /// MCU on the path. Rule 7's count, kept **beside** `bridge` rather than
    /// instead of it — a bridge with zero downstream devices is still muxed
    /// (the conservative reading), so `bridge` and `downstream >= 1` are two
    /// reasons, not one.
    downstream: usize,
}

impl SerialPorts {
    /// Group every `[mcu]` section's `serial:` path and pick the rate it uses.
    ///
    /// A section without a `serial` option (host library, CAN, the `test:`
    /// simulator) is not on a serial path and is skipped. The section list is
    /// the same two families
    /// [`McuIds::reserve_explicit`](super::ids::McuIds::reserve_explicit) scans:
    /// the bare `[mcu]` and every `[mcu <name>]`.
    ///
    /// The rate of a path is the lowest `baud` its sections write, or
    /// [`DEFAULT_BAUD`] when none of them writes one; sections that write
    /// different rates are warned about, and the lowest wins — see
    /// [`effective_baud_for`].
    pub fn classify(config: &ConfigWrapper) -> Self {
        // path → (mcu name, the baud it declares). A `None` baud means the
        // section writes none, or writes one that does not parse: its own
        // factory reports that, so it takes no part in the choice here.
        let mut groups: BTreeMap<String, Vec<(String, Option<u32>)>> = BTreeMap::new();
        // The `[mcu]` names the config makes bridges (port sections' `*_mcu`),
        // read raw like every claim here: a bridge's path is addressed by id
        // even alone (`is_id_addressed`).
        let bridges = bridge_mcus(config);
        // How many downstream devices each bridge has: rule 7's count, read raw
        // like every other claim here.
        let downstream_counts = bridge_downstream_counts(config);
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

        let mut by_path: BTreeMap<String, PathGroup> = BTreeMap::new();
        for (path, members) in groups {
            let baud = effective_baud_for(&path, &members);
            let bridge = members
                .iter()
                .any(|(identifier, _)| bridges.contains(member_name(identifier)));
            let downstream = members
                .iter()
                .filter_map(|(identifier, _)| downstream_counts.get(member_name(identifier)))
                .sum();
            by_path.insert(
                path,
                PathGroup {
                    members: members.into_iter().map(|(name, _)| name).collect(),
                    baud,
                    bridge,
                    downstream,
                },
            );
        }
        Self { by_path }
    }

    /// Whether two or more MCUs name `path`. A path no MCU names is not shared.
    ///
    /// "Shared" is about how many boards are on the wire; it is **not** the
    /// same question as whether the path is addressed by id — a bridge's path is
    /// too ([`SerialPorts::is_id_addressed`]).
    pub fn is_shared(&self, path: &str) -> bool {
        self.peers(path).len() >= 2
    }

    /// Whether `path` is addressed by id: two or more sections name it, **or**
    /// one or more downstream devices hang on a bridge on it, **or** one of its
    /// sections is a bridge with no downstream devices.
    ///
    /// A bridge forwards its downstream devices' frames on the same link, so the
    /// link carries more than one board's traffic even when a single `[mcu]`
    /// names it — the transport must open it with the per-id header, exactly as a
    /// shared one. This is the question the transport decision asks; it is
    /// deliberately not [`SerialPorts::is_shared`], whose answer is about the
    /// number of `[mcu]` sections alone.
    ///
    /// Rule 7 counts downstream devices, and that count is written out here as
    /// its own reason ([`SerialPorts::downstream_devices`]); the `bridge` term
    /// stays as well, because a muxed bridge frames its output **whatever** is
    /// hanging on it — a bridge with zero downstream devices is addressed by id
    /// on purpose, and decoding its output as bare frames would misparse the
    /// per-id header it writes.
    pub fn is_id_addressed(&self, path: &str) -> bool {
        self.is_shared(path)
            || self.downstream_devices(path) >= 1
            || self.by_path.get(path).is_some_and(|group| group.bridge)
    }

    /// How many downstream `[mcu]` devices hang on `path`'s bridge: the
    /// sections that reference a `[uart_device]`/`[can_device]` port whose
    /// `*_mcu` names an MCU on this path. `0` when no `[mcu]` names `path`, or
    /// when its bridge has no downstream devices.
    ///
    /// This is rule 7's count, and the reason
    /// [`SerialPorts::is_id_addressed`] names on its own. It is reported for a
    /// shared port too — the number is about the bridge's ports, not about how
    /// many sections name the path — so a caller can tell a bridge's downstream
    /// fan-out from an empty one.
    pub fn downstream_devices(&self, path: &str) -> usize {
        self.by_path.get(path).map_or(0, |group| group.downstream)
    }

    /// The MCUs that name `path`, in declaration order — one for an exclusive
    /// port, two or more for a shared one. Empty when no `[mcu]` names it.
    pub fn peers(&self, path: &str) -> &[String] {
        self.by_path
            .get(path)
            .map(|group| group.members.as_slice())
            .unwrap_or(&[])
    }

    /// The rate every MCU on `path` talks at: the lowest `baud` its sections
    /// write, or [`DEFAULT_BAUD`] when none of them writes one. `None` when no
    /// `[mcu]` names `path` at all.
    pub fn effective_baud(&self, path: &str) -> Option<u32> {
        self.by_path.get(path).map(|group| group.baud)
    }

    /// Every `serial:` path the config names, each once, in path order.
    ///
    /// The start-up walks this to check the ports before it serves anything
    /// (`klippy::run`); the grouping's own users ask about one path at a time.
    pub fn paths(&self) -> impl Iterator<Item = &str> {
        self.by_path.keys().map(String::as_str)
    }
}

impl PrinterObject for SerialPorts {
    /// The registry reports no status, exactly as the id allocator defines none
    /// (`mcu/ids.rs`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible: nothing here is a status a client can query.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// The single classification, created on first demand.
///
/// The first `[mcu]` factory to load builds it from the whole config and
/// registers it; every later one finds it already there and **does not**
/// re-classify, so the grouping is a load-time constant rather than something
/// each section recomputes. Every `[mcu]` asks for it, so a config with no
/// shared serial port still gets one (its map is simply empty).
///
/// # Errors
/// A registry key that is already taken by a different object.
pub fn ensure(
    printer: &Arc<Printer>,
    config: &ConfigWrapper,
) -> Result<Arc<SerialPorts>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<SerialPorts>(SERIAL_PORTS_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(SerialPorts::classify(config));
    printer.add_object(SERIAL_PORTS_OBJECT, object.clone())?;
    Ok(object)
}

/// Whether two or more `[mcu]` sections name `path` — the shared-path rule on
/// its own, without [`SerialPorts::classify`]'s baud pass and its warning.
///
/// The reset rule (`config/mcu.rs`) needs this answer while parsing a section,
/// before the grouping object exists, and it must not warn: it reads the same
/// two section families `classify` scans, by the same raw `section().get_text`
/// (never a tracked getter), and stops at the second naming section. A wrapper
/// that cannot reach its siblings — a test's small wrapper, or a part reading a
/// stored section back — has no peers, so it answers `false`.
pub fn path_is_shared(config: &ConfigWrapper, path: &str) -> bool {
    let mut naming = 0;
    for section in config
        .sibling("mcu")
        .into_iter()
        .chain(config.sibling_prefix_sections("mcu"))
    {
        if section.section().get_text("serial").as_deref() == Some(path) {
            naming += 1;
            if naming >= 2 {
                return true;
            }
        }
    }
    false
}

/// The `[mcu]` **name** a section identifier carries: the sub, or `mcu` for the
/// bare section — the spelling a port section's `*_mcu` writes.
fn member_name(identifier: &str) -> &str {
    identifier.strip_prefix("mcu ").unwrap_or(identifier)
}

/// The baud a `[mcu]` section contributes: its `baud` option parsed as a
/// positive integer, or `None` when it writes no `baud` or writes one that does
/// not parse. Only sections that write a rate take part in choosing the path's
/// rate; an absent one follows that choice instead of meaning [`DEFAULT_BAUD`]
/// (see [`effective_baud_for`]), and a rate that is not a positive integer is
/// the section's own transport parse's error to report (`config/mcu.rs`).
fn declared_baud(section: &ConfigWrapper) -> Option<u32> {
    let text = section.section().get_text("baud")?;
    match text.trim().parse::<u32>() {
        Ok(baud) if baud > 0 => Some(baud),
        _ => None,
    }
}

/// The rate a path's sections end up at, warning when they wrote more than one.
///
/// No section writes a rate: the path is at [`DEFAULT_BAUD`]. One rate is
/// written: that rate governs the path, including the sections that wrote none.
/// More than one: the path is `warn!`ed about — naming the path, each rate and
/// the MCUs that wrote it, and the rate chosen — and the lowest is used, the
/// rate every MCU on the line can talk at.
///
/// Sections whose rate does not parse are skipped: their own factory reports
/// them.
fn effective_baud_for(path: &str, members: &[(String, Option<u32>)]) -> u32 {
    // The written rates, lowest first, each with the MCUs that wrote them. A
    // `BTreeMap` so the warning lists them in the same order on every run.
    let mut by_rate: BTreeMap<u32, Vec<&str>> = BTreeMap::new();
    for (name, baud) in members {
        if let Some(baud) = baud {
            by_rate.entry(*baud).or_default().push(name.as_str());
        }
    }
    let Some((&lowest, _)) = by_rate.first_key_value() else {
        return DEFAULT_BAUD;
    };
    if by_rate.len() > 1 {
        let written = by_rate
            .iter()
            .map(|(baud, names)| {
                let names = names
                    .iter()
                    .map(|name| format!("'{name}'"))
                    .collect::<Vec<_>>()
                    .join(", ");
                format!("{baud} written by {names}")
            })
            .collect::<Vec<_>>()
            .join("; ");
        warn!(
            "shared serial port '{path}': the MCUs write different bauds ({written}); \
             using the lowest, {lowest}"
        );
    }
    lowest
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::Config;
    use std::io::Write;
    use std::sync::{Arc, Mutex};
    use tracing_subscriber::layer::SubscriberExt;

    /// Parse `text` and classify through its first `[mcu]` section.
    ///
    /// Any `[mcu]`-family wrapper reaches the same sibling sections, so which one
    /// is wrapped does not change the answer.
    fn classify(text: &str) -> SerialPorts {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        SerialPorts::classify(&wrapper)
    }

    /// Classify `text` through its first `[mcu]` section, returning the warnings
    /// that went out alongside the answer.
    ///
    /// Writing different bauds on one path is reported as a warning, which a test
    /// has to hear to know it happened — and it must *not* be heard when the
    /// rates agree, so both directions are asserted through this.
    fn classify_capturing_warnings(text: &str) -> (SerialPorts, String) {
        let capture = LogCapture::default();
        let subscriber = tracing_subscriber::registry()
            .with(tracing_subscriber::filter::LevelFilter::WARN)
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(capture.clone())
                    .with_ansi(false),
            );
        let ports = tracing::subscriber::with_default(subscriber, || classify(text));
        (ports, capture.text())
    }

    /// A byte sink for the fmt layer above. `with_default` keeps the capture off
    /// the process-wide subscriber, so the tests stay independent of each other
    /// and of whatever the caller installed.
    #[derive(Clone, Default)]
    struct LogCapture(Arc<Mutex<Vec<u8>>>);

    impl LogCapture {
        fn text(&self) -> String {
            let bytes = self
                .0
                .lock()
                .expect("the capture lock is not poisoned")
                .clone();
            String::from_utf8(bytes).expect("tracing writes UTF-8")
        }
    }

    impl Write for LogCapture {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0
                .lock()
                .expect("the capture lock is not poisoned")
                .extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogCapture {
        type Writer = LogCapture;

        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }

    /// A path one MCU names is exclusive; `peers` reports that one MCU, at the
    /// default rate it writes no `baud` for.
    #[test]
    fn a_single_mcu_on_a_path_is_exclusive() {
        let ports = classify("[mcu]\nserial: /dev/ttyACM0\n");
        assert!(!ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu"]);
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(DEFAULT_BAUD));

        // A path no MCU names is not shared, and has no peers.
        assert!(!ports.is_shared("/dev/ttyACM1"));
        assert!(ports.peers("/dev/ttyACM1").is_empty());
    }

    /// A path one `[mcu]` names, but which a port section names as its bridge,
    /// is addressed by id even though it is not shared: the bridge's downstream
    /// devices' frames travel it (rule 7). `is_shared` stays false — the two are
    /// different questions.
    #[test]
    fn a_single_bridge_path_is_id_addressed_but_not_shared() {
        let ports = classify(
            "[mcu bridge]\nserial: /dev/ttyACM0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );
        assert!(!ports.is_shared("/dev/ttyACM0"));
        assert!(ports.is_id_addressed("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu bridge"]);
        assert_eq!(ports.downstream_devices("/dev/ttyACM0"), 0);
    }

    /// Rule 7's count: a path one `[mcu]` names, whose bridge carries three
    /// downstream `[mcu]` devices, is addressed by id — the count is 3, and
    /// `is_shared` stays false because only one section names the path itself.
    #[test]
    fn one_mcu_with_three_downstream_devices_is_id_addressed() {
        let ports = classify(
            "[mcu bridge]\nserial: /dev/ttyACM0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [mcu d1]\nuart: uart0\n\
             [mcu d2]\nuart: uart0\n\
             [mcu d3]\nuart: uart0\n",
        );
        assert!(!ports.is_shared("/dev/ttyACM0"));
        assert!(ports.is_id_addressed("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu bridge"]);
        assert_eq!(ports.downstream_devices("/dev/ttyACM0"), 3);
    }

    /// A lone path no port section names as its bridge is addressed by no id at
    /// all: the change adds a reason, it does not widen every path.
    #[test]
    fn a_single_path_without_a_bridge_is_not_id_addressed() {
        let ports = classify("[mcu]\nserial: /dev/ttyACM0\n");
        assert!(!ports.is_shared("/dev/ttyACM0"));
        assert!(!ports.is_id_addressed("/dev/ttyACM0"));
    }

    /// A path no `[mcu]` names has no rate at all: `None`, not a default.
    #[test]
    fn a_path_no_mcu_names_has_no_effective_baud() {
        let ports = classify("[mcu a]\nserial: /dev/ttyACM0\nbaud: 115200\n");
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(115200));
        assert_eq!(ports.effective_baud("/dev/ttyACM1"), None);
    }

    /// Two `[mcu <name>]` sections on one path are a shared group, both named.
    #[test]
    fn two_prefixed_mcus_on_one_path_are_shared() {
        let ports = classify(
            "[mcu a]\nserial: /dev/ttyACM0\n\
             [mcu b]\nserial: /dev/ttyACM0\n",
        );
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
        );
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.peers("/dev/ttyACM0"), ["mcu", "mcu tool"]);
    }

    /// Two different bauds on one path are warned about and the lowest wins: the
    /// config still loads, at the rate every MCU on the line can talk at. The
    /// warning names the path, both rates, both MCUs and the rate chosen.
    #[test]
    fn two_different_bauds_take_the_lowest_and_warn() {
        let (ports, warnings) = classify_capturing_warnings(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: 250000\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 115200\n",
        );
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(115200));
        assert!(
            warnings.contains(
                "shared serial port '/dev/ttyACM0': the MCUs write different bauds \
                 (115200 written by 'mcu b'; 250000 written by 'mcu a'); \
                 using the lowest, 115200"
            ),
            "the warning should name the path, both rates, both MCUs and the choice:\n{warnings}"
        );
    }

    /// One written rate governs the whole path: the section that writes none
    /// follows it instead of colliding with the default, so there is nothing to
    /// warn about — the rate is the written one, default or not.
    #[test]
    fn one_written_baud_governs_the_whole_path() {
        let (ports, warnings) = classify_capturing_warnings(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: 115200\n\
             [mcu b]\nserial: /dev/ttyACM0\n",
        );
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(115200));
        assert!(
            warnings.is_empty(),
            "one written rate is not a disagreement:\n{warnings}"
        );
    }

    /// No written rate anywhere on the path: every MCU is at [`DEFAULT_BAUD`],
    /// which is agreement, not a disagreement to report.
    #[test]
    fn no_written_baud_uses_the_default_rate() {
        let (ports, warnings) = classify_capturing_warnings(
            "[mcu a]\nserial: /dev/ttyACM0\n\
             [mcu b]\nserial: /dev/ttyACM0\n",
        );
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(DEFAULT_BAUD));
        assert!(
            warnings.is_empty(),
            "nothing was written to disagree about:\n{warnings}"
        );
    }

    /// The same written rate on every MCU is agreement: the rate stands and
    /// there is nothing to warn about.
    #[test]
    fn the_same_written_baud_on_every_mcu_is_no_warning() {
        let (ports, warnings) = classify_capturing_warnings(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: 250000\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 250000\n",
        );
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(250_000));
        assert!(
            warnings.is_empty(),
            "one rate is not a disagreement:\n{warnings}"
        );
    }

    /// A rate that does not parse is skipped: the section's own factory owns that
    /// error, so the other MCU's rate is the path's rate rather than a second
    /// error or a disagreement to warn about.
    #[test]
    fn an_unparsable_baud_is_left_to_the_sections_own_factory() {
        let (ports, warnings) = classify_capturing_warnings(
            "[mcu a]\nserial: /dev/ttyACM0\nbaud: fast\n\
             [mcu b]\nserial: /dev/ttyACM0\nbaud: 115200\n",
        );
        assert!(ports.is_shared("/dev/ttyACM0"));
        assert_eq!(ports.effective_baud("/dev/ttyACM0"), Some(115200));
        assert!(
            warnings.is_empty(),
            "an unparsable rate is not a disagreement:\n{warnings}"
        );
    }

    /// A section without `serial:` is ignored: a `host_library` MCU has a path,
    /// but it is not a serial path, so it has no peers and no rate.
    #[test]
    fn sections_without_serial_are_ignored() {
        let ports = classify(
            "[mcu]\nhost_library: /nonexistent/libklipper_host.so\n\
             [mcu tool]\nhost_library: /nonexistent/libklipper_host.so\n",
        );
        assert!(!ports.is_shared("/nonexistent/libklipper_host.so"));
        assert!(ports.peers("/nonexistent/libklipper_host.so").is_empty());
        assert_eq!(
            ports.effective_baud("/nonexistent/libklipper_host.so"),
            None
        );
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
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        SerialPorts::classify(&wrapper);

        assert!(
            access.sections().is_empty(),
            "classify read {:?}, which would exempt them from check_unused",
            access.sections()
        );
    }

    // -----------------------------------------------------------------------
    // `path_is_shared`, the reset rule's silent query.
    // -----------------------------------------------------------------------

    /// `path_is_shared` over `text`, asked through `identifier`'s wrapper the
    /// way the loader hands a `[mcu]` its section.
    fn is_shared(text: &str, identifier: &str, path: &str) -> bool {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section(identifier)
            .unwrap_or_else(|| panic!("the config has [{identifier}]"));
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        path_is_shared(&wrapper, path)
    }

    /// A path one `[mcu]` names is not shared.
    #[test]
    fn a_path_one_mcu_names_is_not_shared() {
        assert!(!is_shared(
            "[mcu]\nserial: /dev/ttyACM0\n",
            "mcu",
            "/dev/ttyACM0"
        ));
    }

    /// Two `[mcu <name>]` sections on one path are shared.
    #[test]
    fn path_is_shared_sees_two_prefixed_mcus() {
        assert!(is_shared(
            "[mcu a]\nserial: /dev/ttyACM0\n[mcu b]\nserial: /dev/ttyACM0\n",
            "mcu a",
            "/dev/ttyACM0"
        ));
    }

    /// Two MCUs on different paths leave each path exclusive.
    #[test]
    fn two_mcus_on_different_paths_are_not_shared() {
        let text = "[mcu a]\nserial: /dev/ttyACM0\n[mcu b]\nserial: /dev/ttyACM1\n";
        assert!(!is_shared(text, "mcu a", "/dev/ttyACM0"));
        assert!(!is_shared(text, "mcu b", "/dev/ttyACM1"));
    }

    /// The bare `[mcu]` and a `[mcu <name>]` naming the same path are shared:
    /// both families are scanned.
    #[test]
    fn path_is_shared_sees_the_bare_section_with_a_prefixed_one() {
        assert!(is_shared(
            "[mcu]\nserial: /dev/ttyACM0\n[mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
            "/dev/ttyACM0"
        ));
    }

    /// A wrapper that cannot reach its siblings has no peers, so it is never
    /// shared.
    #[test]
    fn a_wrapper_without_the_config_is_never_shared() {
        let (config, _) =
            Config::from_text("[mcu]\nserial: /dev/ttyACM0\n").expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let wrapper = ConfigWrapper::untracked(section);
        assert!(!path_is_shared(&wrapper, "/dev/ttyACM0"));
    }

    /// The query reads no option through the tracker: it must not exempt a
    /// sibling `[mcu]` section's options from `check_unused` on its own account.
    #[test]
    fn path_is_shared_records_no_reads() {
        let (config, _) =
            Config::from_text("[mcu]\nserial: /dev/ttyACM0\n[mcu tool]\nserial: /dev/ttyACM0\n")
                .expect("the test config parses");
        let section = config.get_section("mcu").expect("the bare [mcu]");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        assert!(path_is_shared(&wrapper, "/dev/ttyACM0"));
        assert!(
            access.sections().is_empty(),
            "path_is_shared read {:?}, which would exempt them from check_unused",
            access.sections()
        );
    }
}
