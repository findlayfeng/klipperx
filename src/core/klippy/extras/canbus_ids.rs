//! CAN node-id allocation: the host hands every CAN MCU a node id.
//!
//! A direct port of `klippy/extras/canbus_ids.py`, with two klipperx-only
//! differences spelled out below. Upstream's allocator assigns each CAN MCU the
//! node id its firmware answers on (`new_id = len(ids) + NODEID_FIRST`, four
//! upward, step one) and refuses a `canbus_uuid` it has not seen.
//!
//! # When ids are handed out
//!
//! Order is the whole point: an MCU only learns its own node id from the host
//! **as it connects**, but every id must already be known by then — the host
//! writes each MCU's node id over the CAN admin frames when it brings it up, and
//! two MCUs must not be given the same one. So the two halves are split the way
//! upstream splits them (`klippy/mcu.py`):
//!
//! 1. **Registration** happens while the config file is loaded: [`McuObject`]'s
//!    factory registers a uuid the moment it sees `canbus_uuid` on a section
//!    (`mcu.py:784`, in `MCU.__init__`), so the order ids are handed out in is
//!    the order `[mcu]` sections appear in the file. [`add_uuid`] is what the
//!    MCU calls; the loader's early `[mcu]` phase runs before any other section
//!    can connect, so **all** uuids are registered before the first connection.
//! 2. **Assignment** happens at connect time: [`get_nodeid`] is what the MCU
//!    asks for (`mcu.py:860-861`, in `MCUConnectHelper._attach`), and it is the
//!    value written into the firmware's node id.
//!
//! # The object
//!
//! `[canbus_ids]` is a no-argument section and is never written by a user; it is
//! created on demand by the first CAN `[mcu]` ([`ensure`], the way upstream's
//! `printer.load_object(config, 'canbus_ids')` lazily imports and instantiates
//! it). It is declared `phase = early` so a `[canbus_ids]` section, should one
//! be present, loads **before** the `[mcu]` sections that might ask for it —
//! there is one allocator, never two.
//!
//! # Differences from upstream
//!
//! Upstream's `canbus_ids.py` knows only `canbus_uuid` (the interface is passed
//! along but unused by this version). klipperx has a second, pre-existing option
//! on `[mcu]`, `canbus_nodeid`, and keeps it as an explicit override:
//!
//! 1. **`canbus_nodeid` is klipperx-only.** Written on a section, it is the node
//!    id that MCU uses, registered into this allocator so it also counts toward
//!    the next auto-assigned id. Upstream has no such option.
//! 2. **A node id that collides is a config error here**, reported at load time.
//!    Upstream never checks (there is no explicit option to collide with) and
//!    relies on the firmware's `can_id_conflict` check to shut the printer down
//!    after two MCUs are told the same id. klipperx refuses the config instead:
//!    a collision is either an explicit `canbus_nodeid` duplicating an auto id or
//!    another explicit one, and either way no board should be flashed a node id
//!    that was already taken.
//!
//! Auto-assignment is otherwise exactly upstream's: the `n`th registered uuid
//! (counting explicit ones, which take a slot) gets `NODEID_FIRST + n`. So an
//! explicit id shifts the auto sequence only by occupying its own slot, not by
//! changing the formula. There is no upper-bound check, as upstream has none —
//! only the existing `canbus_nodeid` range check on an explicit value.
//!
//! [`McuObject`]: crate::core::klippy::mcu::McuObject

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// `[canbus_ids]` is a no-argument section created on demand. `phase = early` and
// an order below `[mcu]` (10) keep a present `[canbus_ids]` ahead of the `[mcu]`
// sections that register with it; see the module docs.
section!("canbus_ids", order = 5, phase = early, load = load_config);

/// The name this object is registered under.
pub const CANBUS_IDS_OBJECT: &str = "canbus_ids";

/// The first auto-assigned node id. Upstream's `NODEID_FIRST`
/// (`canbus_ids.py:7`): ids 1-3 are left to the host/bridge.
const NODEID_FIRST: u32 = 4;

/// The allocator state, under one lock so a registration is atomic.
struct State {
    /// uuid → the node id it was assigned, in registration order.
    ids: HashMap<[u8; 6], u32>,
    /// node id → the `[mcu]` that claimed it, for a collision's message.
    owners: HashMap<u32, String>,
}

/// The one CAN node-id allocator, registered as `canbus_ids`.
pub struct PrinterCanbusIds {
    state: Mutex<State>,
}

impl PrinterCanbusIds {
    /// An empty allocator.
    pub fn new() -> Self {
        Self {
            state: Mutex::new(State {
                ids: HashMap::new(),
                owners: HashMap::new(),
            }),
        }
    }

    /// Register `uuid` and assign it a node id, returning that id.
    ///
    /// The order ids are handed out in is the order of these calls, which the
    /// `[mcu]` factory makes the file's declaration order. `explicit` is a
    /// `canbus_nodeid` written on the section: it names the id directly instead
    /// of taking the next auto one, but it still takes a slot in the sequence
    /// (so the next auto id is `len(ids) + NODEID_FIRST`, counting it).
    ///
    /// # Errors
    /// Upstream's `Duplicate canbus_uuid` when the uuid is already registered
    /// (`canbus_ids.py:15`), or a config error naming both MCUs when two
    /// sections would end up on one node id (see the module docs).
    pub fn add_uuid(
        &self,
        name: &str,
        uuid: [u8; 6],
        explicit: Option<u32>,
    ) -> Result<u32, ConfigError> {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if state.ids.contains_key(&uuid) {
            return Err(ConfigError::new("Duplicate canbus_uuid".to_string()));
        }
        let new_id = explicit.unwrap_or((state.ids.len() as u32) + NODEID_FIRST);
        if let Some(owner) = state.owners.get(&new_id) {
            return Err(ConfigError::new(match explicit {
                Some(_) => format!(
                    "canbus_ids: node id {new_id} is claimed by MCU '{name}' with \
                     canbus_nodeid but is already used by MCU '{owner}'; set a \
                     different canbus_nodeid"
                ),
                None => format!(
                    "canbus_ids: node id {new_id} is derived for MCU '{name}' but MCU \
                     '{owner}' claims it with canbus_nodeid; set an explicit \
                     canbus_nodeid on one of them"
                ),
            }));
        }
        state.ids.insert(uuid, new_id);
        state.owners.insert(new_id, name.to_string());
        Ok(new_id)
    }

    /// The node id assigned to `uuid`.
    ///
    /// # Errors
    /// Upstream's `Unknown canbus_uuid %s` (`canbus_ids.py:19-21`) when the uuid
    /// was never registered.
    pub fn get_nodeid(&self, uuid: [u8; 6]) -> Result<u32, ConfigError> {
        self.state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .ids
            .get(&uuid)
            .copied()
            .ok_or_else(|| ConfigError::new(format!("Unknown canbus_uuid {}", uuid_hex(uuid))))
    }
}

impl Default for PrinterCanbusIds {
    fn default() -> Self {
        Self::new()
    }
}

impl PrinterObject for PrinterCanbusIds {
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

/// The single `canbus_ids` object, created on first demand.
///
/// Upstream's `printer.load_object(config, 'canbus_ids')` (`mcu.py:784`): the
/// object exists exactly once, whether the config carries a `[canbus_ids]`
/// section or not.
///
/// # Errors
/// A name that is already taken by a different object.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterCanbusIds>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<PrinterCanbusIds>(CANBUS_IDS_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(PrinterCanbusIds::new());
    printer.add_object(CANBUS_IDS_OBJECT, object.clone())?;
    Ok(object)
}

/// The factory `section!` names (`canbus_ids.py:26 def load_config`).
///
/// Returns a fresh object for the loader to register; a `[canbus_ids]` section
/// is declared before `[mcu]`, so [`ensure`] finds this one and does not build a
/// second.
pub fn load_config(
    _config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterCanbusIds::new()))
}

/// `uuid` as the twelve lowercase hex digits a `canbus_uuid` is written with.
fn uuid_hex(uuid: [u8; 6]) -> String {
    uuid.iter().map(|byte| format!("{byte:02x}")).collect()
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

    /// Three uuids registered in declaration order get `NODEID_FIRST` upward,
    /// one step each — upstream's `len(ids) + NODEID_FIRST`.
    #[test]
    fn ids_are_handed_out_in_registration_order_from_four() {
        let ids = PrinterCanbusIds::new();
        assert_eq!(ids.add_uuid("mcu", A, None).unwrap(), 4);
        assert_eq!(ids.add_uuid("mcu tool", B, None).unwrap(), 5);
        assert_eq!(ids.add_uuid("mcu bed", C, None).unwrap(), 6);
        assert_eq!(ids.get_nodeid(A).unwrap(), 4);
        assert_eq!(ids.get_nodeid(B).unwrap(), 5);
        assert_eq!(ids.get_nodeid(C).unwrap(), 6);
    }

    /// An explicit `canbus_nodeid` is used as written and still takes a slot, so
    /// the next auto id counts past it: with explicit 4 and 9 registered first,
    /// a following auto id is `len(ids) + NODEID_FIRST` = 2 + 4 = 6.
    #[test]
    fn an_explicit_nodeid_is_used_and_counts_toward_the_sequence() {
        let ids = PrinterCanbusIds::new();
        assert_eq!(ids.add_uuid("mcu a", A, Some(4)).unwrap(), 4);
        assert_eq!(ids.add_uuid("mcu b", B, Some(9)).unwrap(), 9);
        assert_eq!(ids.add_uuid("mcu c", C, None).unwrap(), 6);
        assert_eq!(ids.get_nodeid(A).unwrap(), 4);
        assert_eq!(ids.get_nodeid(B).unwrap(), 9);
        assert_eq!(ids.get_nodeid(C).unwrap(), 6);
    }

    /// A derived id that would land on an explicit one is a config error at
    /// registration time, naming both MCUs and the id — not a firmware
    /// `can_id_conflict` shutdown later.
    #[test]
    fn a_derived_id_colliding_with_an_explicit_one_is_rejected() {
        let ids = PrinterCanbusIds::new();
        assert_eq!(ids.add_uuid("mcu a", A, Some(6)).unwrap(), 6);
        assert_eq!(ids.add_uuid("mcu b", B, None).unwrap(), 5);
        // len(ids) = 2 → 6, which `mcu a` claimed explicitly.
        let err = ids.add_uuid("mcu c", C, None).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("node id 6"), "{text}");
        assert!(text.contains("mcu a"), "{text}");
        assert!(text.contains("mcu c"), "{text}");
    }

    /// Two explicit ids that collide are rejected too, with both names.
    #[test]
    fn two_explicit_ids_that_collide_are_rejected() {
        let ids = PrinterCanbusIds::new();
        assert_eq!(ids.add_uuid("mcu a", A, Some(7)).unwrap(), 7);
        let err = ids.add_uuid("mcu b", B, Some(7)).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("node id 7"), "{text}");
        assert!(text.contains("mcu a"), "{text}");
        assert!(text.contains("mcu b"), "{text}");
    }

    /// Registering a uuid twice is upstream's error, word for word.
    #[test]
    fn a_duplicate_uuid_is_rejected_with_upstreams_message() {
        let ids = PrinterCanbusIds::new();
        ids.add_uuid("mcu", A, None).unwrap();
        let err = ids.add_uuid("mcu other", A, None).unwrap_err();
        assert_eq!(err.to_string(), "Duplicate canbus_uuid");
    }

    /// Asking for a uuid nobody registered is upstream's error too.
    #[test]
    fn an_unknown_uuid_is_rejected_with_upstreams_message() {
        let ids = PrinterCanbusIds::new();
        let err = ids.get_nodeid(A).unwrap_err();
        assert_eq!(err.to_string(), "Unknown canbus_uuid 11aa22bb33cc");
    }

    // -----------------------------------------------------------------------
    // Through the loader: registration happens as `[mcu]` sections are built.
    // -----------------------------------------------------------------------

    use crate::core::klippy::config::Config;
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::ManualReactor;

    /// Load `text` as a config, returning the printer and the load result.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let result = printer.load_config(&config);
        (printer, result)
    }

    /// The allocator object the loader created.
    fn allocator(printer: &Arc<Printer>) -> Arc<PrinterCanbusIds> {
        printer
            .lookup_object_as::<PrinterCanbusIds>(CANBUS_IDS_OBJECT)
            .expect("the allocator was created")
    }

    /// A three-MCU config hands out 4/5/6 in declaration order: the bare
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
        let ids = allocator(&printer);
        assert_eq!(ids.get_nodeid(A).unwrap(), 4);
        assert_eq!(ids.get_nodeid(B).unwrap(), 5);
        assert_eq!(ids.get_nodeid(C).unwrap(), 6);
    }

    /// An explicit `canbus_nodeid` is honoured and shifts the auto sequence only
    /// by taking its slot; the next auto id is `len(ids) + NODEID_FIRST`.
    #[test]
    fn a_config_honours_an_explicit_nodeid() {
        let (printer, result) = load(
            "[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 9\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("explicit and derived ids do not collide here");
        let ids = allocator(&printer);
        assert_eq!(ids.get_nodeid(A).unwrap(), 9);
        // `len(ids)` is 1 when the second uuid registers → 1 + 4 = 5.
        assert_eq!(ids.get_nodeid(B).unwrap(), 5);
    }

    /// A CAN MCU with no `canbus_nodeid` now loads (the old "does not allocate
    /// one yet" error is gone) and is given an assigned id.
    #[test]
    fn a_can_mcu_without_a_nodeid_loads() {
        let (printer, result) = load("[mcu]\ncanbus_uuid: 11aa22bb33cc\n");
        result.expect("a canbus_uuid alone is enough");
        assert_eq!(allocator(&printer).get_nodeid(A).unwrap(), 4);
    }

    /// A serial MCU neither needs nor creates the allocator.
    #[test]
    fn a_serial_mcu_does_not_use_the_allocator() {
        let (printer, result) = load("[mcu]\nserial: /dev/a\n");
        result.expect("a serial MCU loads without a CAN bus");
        assert!(printer
            .lookup_object_as::<PrinterCanbusIds>(CANBUS_IDS_OBJECT)
            .is_none());
    }

    /// A `[canbus_ids]` section is accepted, and there is still one allocator
    /// whose ids follow the `[mcu]` order (it loads ahead of `[mcu]`).
    #[test]
    fn an_explicit_canbus_ids_section_yields_one_allocator() {
        let (printer, result) = load(
            "[canbus_ids]\n\
             [mcu]\ncanbus_uuid: 11aa22bb33cc\n\
             [mcu tool]\ncanbus_uuid: 0a0b0c0d0e0f\n",
        );
        result.expect("the allocator section loads");
        let ids = allocator(&printer);
        assert_eq!(ids.get_nodeid(A).unwrap(), 4);
        assert_eq!(ids.get_nodeid(B).unwrap(), 5);
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
        assert!(text.contains("node id 6"), "{text}");
        assert!(text.contains("gamma"), "{text}");
        assert!(text.contains("alpha"), "{text}");
    }
}
