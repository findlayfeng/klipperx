//! The MCU-layer id allocator: every `[mcu]` owns a unique id before it talks.
//!
//! A CAN MCU cannot be spoken to until the host has handed it a **node id** over
//! the admin broadcast (`src/generic/canserial.c`), and the number it answers on
//! has to be unique on the bus — so "has an id before it communicates" is a
//! property of the MCU, not of the transport. Upstream keeps the allocator in
//! `[canbus_ids]` (`klippy/extras/canbus_ids.py`): one object, one sequence, ids
//! handed out in `[mcu]` declaration order (`len(ids) + NODEID_FIRST`, four
//! upward). This module is that allocator, generalized to every `[mcu]`
//! transport: the node id of a CAN MCU and the id of a serial or host-lib MCU
//! share **one** sequence, so no two MCUs — whatever they are connected by —
//! can ever be assigned the same number.
//!
//! Unlike upstream's counter, an auto id here is **the lowest free id from
//! `NODEID_FIRST` up**, and every explicit `canbus_nodeid` in the config is
//! reserved before the first `[mcu]` takes one ([`McuIds::reserve_explicit`]).
//! An id therefore no longer depends on how many MCUs were declared before it:
//! an explicit claim is honoured wherever it sits in the file, and an auto id
//! can never land on one — it steps over both the ids already handed out and
//! the ones the config claims explicitly.
//!
//! The CAN flow keeps its concrete shape: a uuid registers (a duplicate is
//! rejected), and an explicit `canbus_nodeid` overrides the sequence. The other
//! transports consume the number differently: serial hands
//! it to the device as the frame-level assignment before identify
//! (`crate::core::klippy::assign`), while `host_library` and the `test:`
//! simulator send nothing yet. Nothing uses an id for *addressing* outside CAN
//! — the frame-header scheme is still to be built. That is the point of moving
//! the id to this layer: every MCU is assigned and gates communication on it
//! (`McuObject::connect` refuses to open without one), while what each
//! transport does with the number is its own.
//!
//! # Lifecycle
//!
//! One allocator per printer, created on demand as `[mcu]` sections build
//! ([`ensure`]). The canonical section/object name is `[mcu_ids]`; the historical
//! `[canbus_ids]` (CAN was the only transport that needed an id) is still
//! accepted and registers the **same** object, so a config written for the old
//! name — CAN mode included — keeps loading. Each `[mcu]` registers in
//! `mcu/object.rs::load_config` — before any of them connects, so the whole
//! sequence is known first — and the returned id is stored on the object;
//! `McuObject::connect` fills it into the parsed config before the transport is
//! opened.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// `phase = early` and an order below `[mcu]` (10) keep a present `[mcu_ids]` /
// `[canbus_ids]` section ahead of the `[mcu]` sections that register with it;
// see the module docs. `[canbus_ids]` is the historical spelling: it loads the
// same allocator but registers the object under the canonical `mcu_ids` key
// (`object = "mcu_ids"`), so both names yield one allocator, never two.
section!("mcu_ids", order = 5, phase = early, load = load_config);
section!(
    "canbus_ids",
    order = 5,
    phase = early,
    load = load_config,
    object = "mcu_ids"
);

/// The key the allocator is registered under (the `[mcu_ids]` section).
///
/// The historical `[canbus_ids]` section also builds this object — its factory
/// declares `object = "mcu_ids"` — so a config that writes the old name still
/// loads and shares the one allocator.
pub const MCU_IDS_OBJECT: &str = "mcu_ids";

/// The first id in the MCU id sequence. Upstream's `NODEID_FIRST`
/// (`canbus_ids.py:7`): ids 1-3 are left to the host/bridge.
const NODEID_FIRST: u32 = 4;

/// The shared bookkeeping.
#[derive(Default)]
struct State {
    /// uuid → assigned id. A CAN MCU is keyed by its uuid because a duplicate
    /// `canbus_uuid` is upstream's error, and the id must be stable per uuid
    /// within a run.
    by_uuid: HashMap<[u8; 6], u32>,
    /// id → owning mcu name. A second explicit `canbus_nodeid` landing on a
    /// taken one is a config error that names both.
    owners: HashMap<u32, String>,
    /// ids the config claims with an explicit `canbus_nodeid`, collected before
    /// any `[mcu]` registers ([`McuIds::reserve_explicit`]). The auto sequence
    /// steps over them, so an explicit claim is honoured whether its section
    /// comes before or after the MCUs that take auto ids.
    reserved: HashSet<u32>,
}

impl State {
    /// The lowest free id from `NODEID_FIRST` up: the first id that is neither
    /// owned already nor reserved by an explicit `canbus_nodeid`. `None` once
    /// every id through `MAX_MCU_ID` is used up.
    fn next_auto(&self) -> Option<u32> {
        (NODEID_FIRST..=MAX_MCU_ID)
            .find(|id| !self.owners.contains_key(id) && !self.reserved.contains(id))
    }
}

/// The mcu layer's single id allocator (one per printer, `Arc`-shared).
///
/// Every `[mcu]` factory registers here while the config file loads; the node id
/// a CAN MCU is administered with and the id a serial MCU is merely assigned are
/// the same sequence, so the two are never handed the same number.
#[derive(Default)]
pub struct McuIds {
    state: Mutex<State>,
}

impl McuIds {
    /// An empty allocator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserve the ids the config claims explicitly with `canbus_nodeid`,
    /// before any `[mcu]` registers.
    ///
    /// An auto id here is the lowest free one from `NODEID_FIRST` up, so the
    /// allocator has to know every explicit claim in advance: a `[mcu]` that
    /// writes `canbus_nodeid: 5` owns 5 whatever its position in the file, and
    /// the MCUs without one take the ids it left free. Without this pre-scan a
    /// `[mcu]` declared before the explicit one would take 5 first and the
    /// explicit claim would be rejected at load.
    ///
    /// The value is read straight from the section's raw parameters rather than
    /// through `get_str`/`get_int`: those record the option as read, which
    /// would exempt `canbus_nodeid` from
    /// [`check_unused`](crate::core::klippy::config::check_unused) and let a
    /// config that has CAN options without a `canbus_uuid` — a serial `[mcu]`
    /// that also writes `canbus_nodeid`, say — pass silently. The option stays
    /// unread here; the section's own factory reads and validates what it
    /// means. A value that does not parse or is outside Klipper's node-id range
    /// is ignored for the same reason: that section's factory reports it.
    ///
    /// Calling this again changes nothing, and a wrapper built without the
    /// whole config (a test wrapper) reserves nothing rather than failing.
    ///
    /// The pre-scan reads the **parsed** config: [`ConfigWrapper::sibling`]
    /// pulls sections straight out of the original
    /// [`Config`](crate::core::klippy::config::Config), before the loader layers
    /// [`Printer::override_config`] over a section
    /// (`crate::core::klippy::load`'s `register`). `canbus_nodeid` therefore must
    /// **not** be overridden with `override_config`: the pre-scan would reserve
    /// the file's value while the section's own factory reads the override, so
    /// the allocator and the MCU would see different ids. The one override today
    /// is `restart_method` (`mcu/object.rs`'s `usb_reset_unusable`), and it never
    /// touches an id.
    pub fn reserve_explicit(&self, config: &ConfigWrapper) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        let sections = config
            .sibling("mcu")
            .into_iter()
            .chain(config.sibling_prefix_sections("mcu"));
        for section in sections {
            if let Some(id) = explicit_nodeid(&section) {
                state.reserved.insert(id);
            }
        }
    }

    /// The next id for an MCU with no transport-specific allocation of its own.
    ///
    /// Serial and host-lib MCUs take this at load. No concrete allocation flow
    /// is implemented for them yet, and nothing consumes the number — it is
    /// assigned so the MCU owns an id before it can communicate, ready for the
    /// transport that later makes it meaningful.
    ///
    /// # Errors
    /// A config error naming the MCU when the whole `NODEID_FIRST`..=`MAX_MCU_ID`
    /// range is already taken.
    pub fn register(&self, name: &str) -> Result<u32, ConfigError> {
        self.insert(name, None, None)
    }

    /// Register a CAN MCU: `uuid`, with an optional explicit `canbus_nodeid`.
    ///
    /// # Errors
    /// Upstream's `Duplicate canbus_uuid` when the uuid already registered
    /// (`canbus_ids.py:15`), or a config error naming both MCUs when the
    /// explicit id is already taken (by another explicit id; an auto id steps
    /// over it) or when the id range is exhausted.
    pub fn register_can(
        &self,
        name: &str,
        uuid: [u8; 6],
        explicit: Option<u32>,
    ) -> Result<u32, ConfigError> {
        self.insert(name, Some(uuid), explicit)
    }

    /// The shared registration: claim the explicit id, or the lowest free one.
    fn insert(
        &self,
        name: &str,
        uuid: Option<[u8; 6]>,
        explicit: Option<u32>,
    ) -> Result<u32, ConfigError> {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(uuid) = uuid {
            if state.by_uuid.contains_key(&uuid) {
                return Err(ConfigError::new("Duplicate canbus_uuid".to_string()));
            }
        }
        let new_id = match explicit {
            Some(new_id) => {
                if let Some(owner) = state.owners.get(&new_id) {
                    return Err(ConfigError::new(format!(
                        "mcu_ids: id {new_id} is claimed by MCU '{name}' with \
                         canbus_nodeid but is already used by MCU '{owner}'; set a \
                         different canbus_nodeid"
                    )));
                }
                new_id
            }
            // An auto id cannot land on a taken one: `next_auto` scans past
            // everything owned and everything reserved by an explicit
            // `canbus_nodeid`, so there is no "derived id collides" error here
            // — only a range that is used up.
            None => match state.next_auto() {
                Some(new_id) => new_id,
                None => {
                    return Err(ConfigError::new(format!(
                        "mcu_ids: no free id for MCU '{name}': every id in \
                         {NODEID_FIRST}..={MAX_MCU_ID} is already used or reserved"
                    )))
                }
            },
        };
        if let Some(uuid) = uuid {
            state.by_uuid.insert(uuid, new_id);
        }
        state.owners.insert(new_id, name.to_string());
        Ok(new_id)
    }
}

/// The explicit `canbus_nodeid` a section writes, read without recording it.
///
/// `None` when the option is absent, does not parse as an id, or is outside
/// `1..=MAX_MCU_ID` — the range [`McuConfig::new`](crate::core::klippy::config::mcu::McuConfig::new)
/// accepts; such a section is rejected by its own factory, so there is nothing
/// to reserve here.
fn explicit_nodeid(section: &ConfigWrapper) -> Option<u32> {
    if !section.has("canbus_nodeid") {
        return None;
    }
    let text = section.section().parameters.get("canbus_nodeid")?;
    match text.as_str_ref()?.trim().parse::<u32>() {
        Ok(id) if (1..=MAX_MCU_ID).contains(&id) => Some(id),
        _ => None,
    }
}

impl PrinterObject for McuIds {
    /// The allocator reports no status, exactly as upstream's `PrinterCANBus`
    /// defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible: upstream's `objects/list` keeps only objects with a
    /// `get_status`, and this one has none (it is not in the list either).
    fn is_queryable(&self) -> bool {
        false
    }
}

/// The single allocator, created on first demand.
///
/// Upstream's `printer.load_object(config, 'canbus_ids')` (`mcu.py:784`): the
/// object exists exactly once, whether the config carries an explicit section
/// (under either its `[mcu_ids]` name or the historical `[canbus_ids]`) or not.
/// Every `[mcu]` factory asks for it, so a config with no CAN MCU gets one too —
/// the serial MCU's id comes from the same sequence.
///
/// # Errors
/// A name that is already taken by a different object.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<McuIds>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<McuIds>(MCU_IDS_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(McuIds::new());
    printer.add_object(MCU_IDS_OBJECT, object.clone())?;
    Ok(object)
}

/// The factory `section!` names (`canbus_ids.py:26 def load_config`).
///
/// Returns a fresh object for the loader to register; a `[mcu_ids]` section — or
/// a `[canbus_ids]` one, which registers under the same key — is declared before
/// `[mcu]`, so [`ensure`] finds this one and does not build a second.
pub fn load_config(
    _config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(McuIds::new()))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const A: [u8; 6] = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
    const B: [u8; 6] = [0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f];
    const C: [u8; 6] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];

    /// CAN uuids registered with no explicit id get `NODEID_FIRST` upward, one
    /// step each: the auto id is the lowest free one.
    #[test]
    fn ids_are_handed_out_in_registration_order_from_four() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu", A, None).unwrap(), 4);
        assert_eq!(ids.register_can("mcu tool", B, None).unwrap(), 5);
        assert_eq!(ids.register_can("mcu bed", C, None).unwrap(), 6);
    }

    /// An explicit `canbus_nodeid` is used as written and the auto sequence
    /// steps over it: with 4 and 9 claimed explicitly, the next auto id is the
    /// lowest free one, 5 — not a count past them.
    #[test]
    fn an_explicit_nodeid_is_used_and_the_auto_sequence_steps_over_it() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(4)).unwrap(), 4);
        assert_eq!(ids.register_can("mcu b", B, Some(9)).unwrap(), 9);
        assert_eq!(ids.register_can("mcu c", C, None).unwrap(), 5);
    }

    /// An auto id never lands on an explicit one: it scans past it and fills the
    /// lowest free id instead, so there is no load-time collision error here.
    #[test]
    fn an_auto_id_avoids_an_explicitly_claimed_id() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(6)).unwrap(), 6);
        // 4 and 5 are free even though 6 is claimed explicitly.
        assert_eq!(ids.register_can("mcu b", B, None).unwrap(), 4);
        assert_eq!(ids.register_can("mcu c", C, None).unwrap(), 5);
    }

    /// Without a pre-scan a bare allocator still starts at four and takes the
    /// lowest free id, so it fills a hole an explicit claim left behind.
    #[test]
    fn a_bare_allocator_takes_the_lowest_free_id_from_four() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(4)).unwrap(), 4);
        assert_eq!(ids.register_can("mcu b", B, Some(6)).unwrap(), 6);
        assert_eq!(ids.register("mcu c").unwrap(), 5);
        assert_eq!(ids.register("mcu d").unwrap(), 7);
    }

    /// A second explicit id that claims a taken one is rejected too, with both
    /// names — the one explicit-claim error the auto scan cannot avoid.
    #[test]
    fn two_explicit_ids_that_collide_are_rejected() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(7)).unwrap(), 7);
        let err = ids.register_can("mcu b", B, Some(7)).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("id 7"), "{text}");
        assert!(text.contains("mcu a"), "{text}");
        assert!(text.contains("mcu b"), "{text}");
    }

    /// Registering a uuid twice is upstream's error, word for word.
    #[test]
    fn a_duplicate_uuid_is_rejected_with_upstreams_message() {
        let ids = McuIds::new();
        ids.register_can("mcu", A, None).unwrap();
        let err = ids.register_can("mcu other", A, None).unwrap_err();
        assert_eq!(err.to_string(), "Duplicate canbus_uuid");
    }

    /// A non-CAN MCU takes ids from the **same** sequence as CAN: the two kinds
    /// never share a number (the point of the unified allocator).
    #[test]
    fn serial_and_can_share_one_sequence() {
        let ids = McuIds::new();
        assert_eq!(ids.register("mcu serial").unwrap(), 4);
        assert_eq!(ids.register_can("mcu can", A, None).unwrap(), 5);
        assert_eq!(ids.register("mcu host").unwrap(), 6);
        // An explicit CAN id is skipped, not counted: with 4/5/6/9 taken, the
        // next auto id is the lowest free one, 7.
        assert_eq!(ids.register_can("mcu tip", B, Some(9)).unwrap(), 9);
        assert_eq!(ids.register("mcu tail").unwrap(), 7);
    }

    /// A non-CAN MCU also steps over an explicit CAN id: with 5 claimed
    /// explicitly, the serial MCU takes 4.
    #[test]
    fn a_serial_id_avoids_an_explicit_can_id() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu can", A, Some(5)).unwrap(), 5);
        assert_eq!(ids.register("mcu serial").unwrap(), 4);
    }

    // -----------------------------------------------------------------------
    // Through the loader: registration happens as `[mcu]` sections are built.
    // -----------------------------------------------------------------------

    use crate::core::klippy::config::AccessTracking;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::mcu::object::McuObject;
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::ManualReactor;

    /// Load `text` as a config, returning the printer and the load result.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let result = printer.load_config(&config);
        (printer, result)
    }

    /// Whatever it was assigned, an `[mcu]` object carries its id.
    fn assigned_id(printer: &Arc<Printer>, name: &str) -> u32 {
        printer
            .lookup_object_as::<McuObject>(name)
            .expect("the section registered")
            .id()
            .unwrap()
    }

    /// A three-CAN-MCU config hands out 4/5/6 in declaration order: the bare
    /// `[mcu]` first, then the prefixes in file order, the way upstream's
    /// `add_printer_objects` registers them (`klippy/mcu.py:1240-1243`).
    #[test]
    fn a_config_assigns_ids_in_declaration_order() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n\
             [mcu bed]\ncanbus_uuid: 010203040506\n",
        );
        result.expect("a canbus_uuid alone is enough");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
        assert_eq!(assigned_id(&printer, "mcu bed"), 6);
    }

    /// An explicit `canbus_nodeid` is honoured and the auto sequence steps over
    /// it: the MCU without one gets the lowest free id, 4 — the claim of 9 does
    /// not move it.
    #[test]
    fn a_config_honours_an_explicit_nodeid() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 9\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("explicit and auto ids do not collide here");
        assert_eq!(assigned_id(&printer, "mcu"), 9);
        // The lowest free id: 4, not 5.
        assert_eq!(assigned_id(&printer, "mcu tool"), 4);
    }

    /// The auto id is the lowest free one, so an explicit `canbus_nodeid` is
    /// honoured wherever its section sits: whether the MCU that takes the auto
    /// id is declared before or after the one that claims 5, they get 4 and 5.
    #[test]
    fn an_explicit_nodeid_is_honoured_whatever_the_declaration_order() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\ncanbus_nodeid: 5\n",
        );
        result.expect("the explicit claim is reserved before ids are handed out");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);

        // The same two sections, in the other order, get the same ids.
        let (printer, result) = load(
            "[mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\ncanbus_nodeid: 5\n\
             [mcu]\ncanbus_uuid: 11aa22bb33cc\n",
        );
        result.expect("the explicit claim is reserved before ids are handed out");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
    }

    /// An explicit `canbus_nodeid` is reserved when a *prefix* section that
    /// takes an auto id is declared before it: `[mcu b]` registers first and
    /// would grab 4, but the pre-scan already reserved the 4 `[mcu a]` claims,
    /// so `[mcu b]` steps over it and the explicit claim is honoured. Without the
    /// pre-scan this config fails at load with the id-4 collision error.
    #[test]
    fn an_explicit_claim_is_reserved_before_an_earlier_auto_id() {
        let (printer, result) = load(
            "[mcu b]\ncanbus_uuid: 0a0b0c0d0e0f\n\
             [mcu a]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 4\n",
        );
        result.expect("the early prefix takes the id the later claim left free");
        assert_eq!(assigned_id(&printer, "mcu b"), 5);
        assert_eq!(assigned_id(&printer, "mcu a"), 4);

        // The same two prefix sections, written the other way round, get the
        // same ids. This half pins order-independence between prefix sections;
        // on its own it does **not** discriminate the pre-scan (the explicit
        // section now registers first), only the half above does.
        let (printer, result) = load(
            "[mcu a]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 4\n\
             [mcu b]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("both prefix sections load in either order");
        assert_eq!(assigned_id(&printer, "mcu b"), 5);
        assert_eq!(assigned_id(&printer, "mcu a"), 4);
    }

    /// A reserved id below `NODEID_FIRST` costs the auto sequence nothing: with
    /// `canbus_nodeid: 2` claimed, the auto MCU still gets 4.
    #[test]
    fn a_reserved_id_below_the_sequence_does_not_move_the_auto_id() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 2\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("a nodeid below the sequence is valid");
        assert_eq!(assigned_id(&printer, "mcu"), 2);
        assert_eq!(assigned_id(&printer, "mcu tool"), 4);
    }

    /// The pre-scan reads `canbus_nodeid` straight from the raw section
    /// parameters, so it is not recorded as read — the section's own factory
    /// stays the one that reads and validates it. Reading it with `get_str`
    /// instead would show up in the access tracking, which is the only way to
    /// tell the two apart; and the reserved id is really in effect, so the auto
    /// sequence steps over it.
    #[test]
    fn the_pre_scan_reads_the_explicit_id_without_recording_it() {
        let (config, _) = Config::from_text("[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 5\n")
            .expect("the config parses");
        let access = AccessTracking::shared();
        let section = config.get_section("mcu").expect("the section is there");
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        let ids = McuIds::new();
        ids.reserve_explicit(&wrapper);
        assert!(
            !access.contains("mcu", "canbus_nodeid"),
            "the pre-scan must not record canbus_nodeid as read"
        );

        // The reservation is in effect: 5 is skipped, so the second auto id is 6.
        assert_eq!(ids.register("mcu auto").unwrap(), 4);
        assert_eq!(ids.register("mcu auto2").unwrap(), 6);
    }

    /// A CAN MCU with no `canbus_nodeid` loads (the old "does not allocate one
    /// yet" error is gone) and is given an assigned id.
    #[test]
    fn a_can_mcu_without_a_nodeid_loads() {
        let (printer, result) = load("[mcu]\ncanbus_uuid: 11aa22bb33cc\n");
        result.expect("a canbus_uuid alone is enough");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
    }

    /// A serial MCU now registers too: it gets the next id from the same
    /// sequence, and brings the allocator into existence.
    #[test]
    fn a_serial_mcu_gets_an_id_from_the_same_sequence() {
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n\
             [mcu bed]\nserial: /dev/b\n",
        );
        result.expect("mixed transports load");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
        assert_eq!(assigned_id(&printer, "mcu bed"), 6);
    }

    /// The historical `[canbus_ids]` name still loads — CAN mode compat — and
    /// there is still one allocator whose ids follow the `[mcu]` order, but the
    /// object it builds is registered under the canonical `mcu_ids` key.
    #[test]
    fn an_explicit_canbus_ids_section_is_accepted_and_one_allocator() {
        let (printer, result) = load(
            "[canbus_ids]\n\
             [mcu]\ncanbus_uuid: 11aa22bb33cc\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("the legacy allocator section loads");
        assert!(
            printer.lookup_object_as::<McuIds>("canbus_ids").is_none(),
            "the legacy section registers under the canonical key, not its own"
        );
        assert!(printer.lookup_object_as::<McuIds>("mcu_ids").is_some());
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
    }

    /// The canonical `[mcu_ids]` section loads a fresh allocator, and `ensure`
    /// from the `[mcu]` sections finds it rather than building a second.
    #[test]
    fn an_explicit_mcu_ids_section_yields_one_allocator() {
        let (printer, result) = load(
            "[mcu_ids]\n\
             [mcu]\ncanbus_uuid: 11aa22bb33cc\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("the allocator section loads");
        assert_eq!(assigned_id(&printer, "mcu"), 4);
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
    }

    /// Two sections claiming the same `canbus_nodeid` are rejected at load,
    /// naming both MCUs and the id — the one collision the auto scan cannot
    /// step around.
    #[test]
    fn a_config_rejects_two_sections_claiming_one_nodeid() {
        let (_printer, result) = load(
            "[mcu alpha]\ncanbus_uuid: 010203040506\ncanbus_nodeid: 6\n\
             [mcu beta]\ncanbus_uuid: 0a0b0c0d0e0f\ncanbus_nodeid: 6\n",
        );
        let err = result.expect_err("the second claim on id 6 is a config error");
        let text = err.to_string();
        assert!(text.contains("id 6"), "{text}");
        assert!(text.contains("beta"), "{text}");
        assert!(text.contains("alpha"), "{text}");
    }

    /// The sequence is finite: once every id from `NODEID_FIRST` through
    /// `MAX_MCU_ID` is taken, the next MCU is refused instead of being
    /// handed an id off the end. 252 ids fit — `0xff - 4 + 1`. The message
    /// spells out the range and that both used and reserved ids exhaust it.
    #[test]
    fn the_allocator_reports_an_exhausted_id_range() {
        let ids = McuIds::new();
        for _ in NODEID_FIRST..=MAX_MCU_ID {
            ids.register("mcu").expect("an id is free");
        }
        let err = ids.register("mcu one too many").unwrap_err();
        assert_eq!(
            err.to_string(),
            format!(
                "mcu_ids: no free id for MCU 'mcu one too many': every id in \
                 {NODEID_FIRST}..={MAX_MCU_ID} is already used or reserved"
            )
        );
    }

    /// Writing both section names registers the same object key twice, which
    /// the printer refuses — one allocator, and the config is told so.
    #[test]
    fn writing_both_section_names_is_a_config_error() {
        let (_printer, result) = load("[mcu_ids]\n[canbus_ids]\n");
        let err = result.expect_err("the second section collides on the object key");
        let text = err.to_string();
        assert!(text.contains("already created"), "{text}");
        assert!(text.contains("mcu_ids"), "{text}");
    }
}
