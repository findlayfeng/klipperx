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
//! The CAN flow keeps its concrete shape: a uuid registers (a duplicate is
//! rejected), an explicit `canbus_nodeid` overrides the sequence and still takes
//! a slot, and a derived id landing on a taken one is a load-time config error
//! naming both MCUs (upstream relies on the firmware's `can_id_conflict` stop
//! instead). The other transports consume the number differently: serial hands
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

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

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

/// The first id in the sequence. Upstream's `NODEID_FIRST` (`canbus_ids.py:7`):
/// ids 1-3 are left to the host/bridge.
const NODEID_FIRST: u32 = 4;

/// The shared bookkeeping.
#[derive(Default)]
struct State {
    /// uuid → assigned id. A CAN MCU is keyed by its uuid because a duplicate
    /// `canbus_uuid` is upstream's error, and the id must be stable per uuid
    /// within a run.
    by_uuid: HashMap<[u8; 6], u32>,
    /// id → owning mcu name. A derived id landing on a taken one — an explicit
    /// `canbus_nodeid`, or a second explicit id — is a config error that names
    /// both.
    owners: HashMap<u32, String>,
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

    /// The next id in the shared sequence: `count + NODEID_FIRST`, upstream's
    /// `len(ids) + NODEID_FIRST` with `count` counting **every** registered MCU,
    /// whatever its transport.
    fn next_auto(count: usize) -> u32 {
        count as u32 + NODEID_FIRST
    }

    /// The next id for an MCU with no transport-specific allocation of its own.
    ///
    /// Serial and host-lib MCUs take this at load. No concrete allocation flow
    /// is implemented for them yet, and nothing consumes the number — it is
    /// assigned so the MCU owns an id before it can communicate, ready for the
    /// transport that later makes it meaningful.
    ///
    /// # Errors
    /// A config error naming both MCUs when the derived id lands on one that is
    /// already taken (only an explicit `canbus_nodeid` can claim it in advance).
    pub fn register(&self, name: &str) -> Result<u32, ConfigError> {
        self.insert(name, None, None)
    }

    /// Register a CAN MCU: `uuid`, with an optional explicit `canbus_nodeid`.
    ///
    /// # Errors
    /// Upstream's `Duplicate canbus_uuid` when the uuid already registered
    /// (`canbus_ids.py:15`), or a config error naming both MCUs when the id this
    /// registers would land on one that is already taken.
    pub fn register_can(
        &self,
        name: &str,
        uuid: [u8; 6],
        explicit: Option<u32>,
    ) -> Result<u32, ConfigError> {
        self.insert(name, Some(uuid), explicit)
    }

    /// The shared registration: claim the next id, or the explicit one.
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
        let new_id = explicit.unwrap_or(Self::next_auto(state.owners.len()));
        if let Some(owner) = state.owners.get(&new_id) {
            return Err(ConfigError::new(match (explicit, uuid) {
                (Some(_), _) => format!(
                    "mcu_ids: id {new_id} is claimed by MCU '{name}' with \
                     canbus_nodeid but is already used by MCU '{owner}'; set a \
                     different canbus_nodeid"
                ),
                (None, Some(_)) => format!(
                    "mcu_ids: id {new_id} is derived for MCU '{name}' but MCU \
                     '{owner}' claims it with canbus_nodeid; set an explicit \
                     canbus_nodeid on one of them"
                ),
                (None, None) => format!(
                    "mcu_ids: id {new_id} is the next id for MCU '{name}' but is \
                     already used by MCU '{owner}', which claims it with \
                     canbus_nodeid; set a different canbus_nodeid on that MCU"
                ),
            }));
        }
        if let Some(uuid) = uuid {
            state.by_uuid.insert(uuid, new_id);
        }
        state.owners.insert(new_id, name.to_string());
        Ok(new_id)
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

    /// CAN uuids registered in declaration order get `NODEID_FIRST` upward, one
    /// step each — upstream's `len(ids) + NODEID_FIRST`.
    #[test]
    fn ids_are_handed_out_in_registration_order_from_four() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu", A, None).unwrap(), 4);
        assert_eq!(ids.register_can("mcu tool", B, None).unwrap(), 5);
        assert_eq!(ids.register_can("mcu bed", C, None).unwrap(), 6);
    }

    /// An explicit `canbus_nodeid` is used as written and still takes a slot, so
    /// the next auto id counts past it: with explicit 4 and 9 registered first,
    /// a following auto id is `count + NODEID_FIRST` = 2 + 4 = 6.
    #[test]
    fn an_explicit_nodeid_is_used_and_counts_toward_the_sequence() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(4)).unwrap(), 4);
        assert_eq!(ids.register_can("mcu b", B, Some(9)).unwrap(), 9);
        assert_eq!(ids.register_can("mcu c", C, None).unwrap(), 6);
    }

    /// A derived id that would land on an explicit one is a config error at
    /// registration time, naming both MCUs and the id — not a firmware
    /// `can_id_conflict` shutdown later.
    #[test]
    fn a_derived_id_colliding_with_an_explicit_one_is_rejected() {
        let ids = McuIds::new();
        assert_eq!(ids.register_can("mcu a", A, Some(6)).unwrap(), 6);
        assert_eq!(ids.register_can("mcu b", B, None).unwrap(), 5);
        // count = 2 → 6, which `mcu a` claimed explicitly.
        let err = ids.register_can("mcu c", C, None).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("id 6"), "{text}");
        assert!(text.contains("mcu a"), "{text}");
        assert!(text.contains("mcu c"), "{text}");
    }

    /// Two explicit ids that collide are rejected too, with both names.
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

    /// A non-CAN MCU takes the next id from the **same** sequence as CAN: the
    /// two kinds never share a number (the point of the unified allocator).
    #[test]
    fn serial_and_can_share_one_sequence() {
        let ids = McuIds::new();
        assert_eq!(ids.register("mcu serial").unwrap(), 4);
        assert_eq!(ids.register_can("mcu can", A, None).unwrap(), 5);
        assert_eq!(ids.register("mcu host").unwrap(), 6);
        // An explicit CAN id still counts toward the sequence for the auto ones:
        // after four registrations the next auto id is 4 + 4 = 8.
        assert_eq!(ids.register_can("mcu tip", B, Some(9)).unwrap(), 9);
        assert_eq!(ids.register("mcu tail").unwrap(), 8);
    }

    /// A non-CAN derived id that lands on an explicit CAN id is a config error
    /// naming both.
    #[test]
    fn a_serial_derived_id_colliding_with_an_explicit_can_id_is_rejected() {
        let ids = McuIds::new();
        // The explicit 5 claims the slot first; the serial MCU then registers
        // with count = 1 → next_auto = 5, which lands on it.
        assert_eq!(ids.register_can("mcu can", A, Some(5)).unwrap(), 5);
        let err = ids.register("mcu serial").unwrap_err();
        let text = err.to_string();
        assert!(text.contains("id 5"), "{text}");
        assert!(text.contains("mcu can"), "{text}");
        assert!(text.contains("mcu serial"), "{text}");
    }

    // -----------------------------------------------------------------------
    // Through the loader: registration happens as `[mcu]` sections are built.
    // -----------------------------------------------------------------------

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

    /// An explicit `canbus_nodeid` is honoured and shifts the auto sequence only
    /// by taking its slot; the next auto id is `count + NODEID_FIRST`.
    #[test]
    fn a_config_honours_an_explicit_nodeid() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 9\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("explicit and derived ids do not collide here");
        assert_eq!(assigned_id(&printer, "mcu"), 9);
        // count is 1 when the second uuid registers → 1 + 4 = 5.
        assert_eq!(assigned_id(&printer, "mcu tool"), 5);
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

    /// A collision through a real config is rejected at load time, naming both
    /// MCUs, instead of reaching the firmware's `can_id_conflict` shutdown.
    #[test]
    fn a_collision_through_a_config_is_rejected_at_load() {
        let (_printer, result) = load(
            "[mcu alpha]\ncanbus_uuid: 010203040506\ncanbus_nodeid: 6\n\
             [mcu beta]\ncanbus_uuid: 0a0b0c0d0e0f\n\
             [mcu gamma]\ncanbus_uuid: 11aa22bb33cc\n",
        );
        let err = result.expect_err("the collision is a config error");
        let text = err.to_string();
        assert!(text.contains("id 6"), "{text}");
        assert!(text.contains("gamma"), "{text}");
        assert!(text.contains("alpha"), "{text}");
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
