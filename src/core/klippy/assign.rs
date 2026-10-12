//! The MCU id ↔ arbitration id mapping, shared by CAN and shared serial.
//!
//! An MCU that shares a bus with other devices is addressed by an id: CAN gets
//! one from its admin frame on `0x3f0` (`interface/devices/canserial.rs`,
//! `CMD_SET_NODEID`), and a **shared** serial port prefixes every unit on the
//! wire with one (`interface/devices/shared_serial.rs`). Both transports take
//! that id from this module and from nowhere else — the mapping is the only
//! thing they have in common.
//!
//! * [`arb_id`] — the id an MCU listens on: `0x100 + 2 * mcu_id`.
//! * [`reply_arb_id`] — the id it answers on: [`arb_id`] plus one.
//! * [`mcu_id_from_arb`] — either direction decoded back to the same `mcu_id`.
//! * [`is_reply_arb`] — the low bit that tells the two directions apart.
//!
//! The domain is `1..=MAX_MCU_ID` (`0xff`); every other value has no mapping,
//! so a caller can neither build nor decode an id the wire format cannot carry.
//!
//! # Where the rest lives
//!
//! Everything transport-specific stays with its transport:
//!
//! * The **serial** side — the `KXID` pseudo-frame that carries the assignment,
//!   its VLQ frame-header codec, and the synchronous handshake — is
//!   `interface/devices/shared_serial.rs`.
//! * The **CAN** side is its own admin frame (`0x3f0` / `CMD_SET_NODEID`) in
//!   `interface/devices/canserial.rs`, which exists on the bus itself rather
//!   than inside the byte stream.

use crate::core::klippy::config::mcu::MAX_MCU_ID;

/// Base of the arbitration-id range, matching CAN's node mapping
/// (`interface/devices/canserial.rs`): `0x100 + 2 * mcu_id`.
const ARB_BASE: u32 = 0x100;

/// The arbitration id an MCU listens on: `0x100 + 2 * mcu_id`.
///
/// `None` for an id outside `1..=MAX_MCU_ID` (`0xff`), so a caller cannot ask
/// for an id the wire format cannot carry.
pub fn arb_id(mcu_id: u32) -> Option<u32> {
    if (1..=MAX_MCU_ID).contains(&mcu_id) {
        Some(ARB_BASE + 2 * mcu_id)
    } else {
        None
    }
}

/// The arbitration id an MCU answers on: [`arb_id`] plus one.
pub fn reply_arb_id(mcu_id: u32) -> Option<u32> {
    arb_id(mcu_id).map(|arb| arb + 1)
}

/// Decode an arbitration id back to the `mcu_id` it names.
///
/// Both directions decode to the same id: the listener id (`0x100 + 2n`, even)
/// and the answer id (`0x101 + 2n`, odd) differ only in the low bit, so
/// `offset / 2` recovers `n` either way. `None` outside the assigned range —
/// in particular an id that would name `0` (`0x100`, `0x101`) or one past
/// `MAX_MCU_ID` (`0x300` and up).
pub fn mcu_id_from_arb(arb: u32) -> Option<u32> {
    let offset = arb.checked_sub(ARB_BASE)?;
    if !(2..=2 * MAX_MCU_ID + 1).contains(&offset) {
        return None;
    }
    Some(offset / 2)
}

/// Whether `arb` is an answer id — the odd half of the mapping.
///
/// The device→host direction uses [`reply_arb_id`]; this is how a router
/// reading a header tells that apart from the host→device [`arb_id`].
pub fn is_reply_arb(arb: u32) -> bool {
    arb.checked_sub(ARB_BASE)
        .is_some_and(|offset| offset % 2 == 1)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arb_id_covers_exactly_one_to_max_mcu_id() {
        // The mapping CAN uses (`interface/devices/canserial.rs`).
        assert_eq!(arb_id(1), Some(0x102));
        assert_eq!(arb_id(255), Some(0x2fe));
        for id in 1..=MAX_MCU_ID {
            assert_eq!(arb_id(id), Some(0x100 + 2 * id), "listener id for {id}");
        }

        // Ids outside `1..=MAX_MCU_ID` cannot be built.
        assert_eq!(arb_id(0), None);
        assert_eq!(arb_id(0x100), None);
        assert_eq!(arb_id(MAX_MCU_ID + 1), None);
    }

    #[test]
    fn reply_arb_id_is_one_past_the_listener_id() {
        assert_eq!(reply_arb_id(1), Some(0x103));
        assert_eq!(reply_arb_id(255), Some(0x2ff));
        for id in 1..=MAX_MCU_ID {
            assert_eq!(
                reply_arb_id(id),
                arb_id(id).map(|arb| arb + 1),
                "answer id for {id}"
            );
        }

        assert_eq!(reply_arb_id(0), None);
        assert_eq!(reply_arb_id(MAX_MCU_ID + 1), None);
    }

    #[test]
    fn mcu_id_from_arb_decodes_both_directions_over_the_range() {
        for id in 1..=MAX_MCU_ID {
            let tx = arb_id(id).unwrap();
            let rx = reply_arb_id(id).unwrap();
            assert_eq!(mcu_id_from_arb(tx), Some(id), "listener id for {id}");
            assert_eq!(mcu_id_from_arb(rx), Some(id), "answer id for {id}");
        }

        // Ids that would name 0 (`0x100`, `0x101`) and ids past the bound
        // (`0x300` up) do not decode.
        assert_eq!(mcu_id_from_arb(0x100), None);
        assert_eq!(mcu_id_from_arb(0x101), None);
        assert_eq!(mcu_id_from_arb(0x300), None);
        assert_eq!(mcu_id_from_arb(0x301), None);
        assert_eq!(mcu_id_from_arb(0), None);
    }

    #[test]
    fn is_reply_arb_ids_the_odd_half() {
        for id in 1..=MAX_MCU_ID {
            assert!(
                !is_reply_arb(arb_id(id).unwrap()),
                "listener id for {id} is not a reply"
            );
            assert!(
                is_reply_arb(reply_arb_id(id).unwrap()),
                "answer id for {id} is a reply"
            );
        }

        // A concrete pair, and ids below the base that belong to no direction.
        assert!(is_reply_arb(0x103));
        assert!(!is_reply_arb(0x102));
        assert!(!is_reply_arb(0x100));
        assert!(!is_reply_arb(0));
    }
}
