//! The MCU configuration phase: object ids, config commands, and the CRC that
//! says whether a firmware already has this configuration.
//!
//! Firmware is configured in one shot. Every resource (`digital_out`, `pwm`,
//! `adc`, an SPI device, …) first reserves an **oid** and then announces itself
//! with a `config_*` command; none of that reaches the wire until the whole
//! configuration is known, because the firmware wants `allocate_oids` first and
//! the host wants one CRC covering everything. [`ConfigBuilder`] is where those
//! commands accumulate, and [`ConfigBuilder::configure`] is the handshake that
//! sends them.
//!
//! # The shape of a configuration
//!
//! Upstream keeps three lists (`MCUConfigHelper`, `klippy/mcu.py:979-1143`):
//!
//! | list | when it is sent | examples |
//! |---|---|---|
//! | `config` | only when the firmware is not configured with this CRC | `config_digital_out`, `config_spi`, `finalize_config` |
//! | `restart` | on every connect, even a reuse | `update_digital_out` (restore start values) |
//! | `init` | on every connect, after the other two | periodic-query setup, startup SPI writes |
//!
//! A build is:
//!
//! 1. the **config callbacks** run, so objects can add the commands they could
//!    not add at construction (they need `CLOCK_FREQ` and the dictionary, which
//!    only exist after identify);
//! 2. `allocate_oids count=N` is prepended, `N` being the final oid count;
//! 3. the `config` list is encoded and hashed into a 32-bit CRC;
//! 4. `finalize_config crc=…` is appended to the `config` list.
//!
//! # Connection
//!
//! [`ConfigBuilder::configure`] asks the firmware what it has:
//!
//! * not configured → send `config` + `init`, then ask again;
//! * configured and the CRC matches → send `restart` + `init` only;
//! * stopped, or configured with a different CRC → **reset it and configure**.
//!   `config_reset` clears the CRC, the oids and the shutdown latch
//!   (`src/basecmd.c:262`), and an emergency stop comes first when the firmware
//!   is still running, because `config_reset` only accepts to run while stopped.
//!   A firmware without `config_reset` (the command is declared per board) is
//!   refused instead, with a message pointing at a power cycle or the restart
//!   loop (TODO D2). Upstream resets the same way, one process later, through
//!   its restart helper — which is why `mcu/object.rs` binds its shutdown events
//!   only *after* this handshake: a reset this host performs is not a
//!   spontaneous stop.
//!
//! # The CRC
//!
//! Upstream hashes the *text* of the configuration commands, newline-joined
//! (`klippy/mcu.py:1017-1019`). This host has no command text — commands are
//! typed (name + [`ArgValue`]s) — so the hash is taken over the **encoded
//! payload bytes** of the `config` list in order. The firmware never computes
//! the CRC, it only stores and returns the 32-bit value, so the only property
//! that matters is that the same configuration hashes the same way across runs;
//! wire bytes satisfy that at least as well as formatted text. A host swap
//! between this and upstream Klipper will see different CRCs and simply
//! reconfigure, which is the safe direction.
//!
//! # Not here
//!
//! * **Pin name rewriting.** Upstream resolves pin aliases and reservations by
//!   rewriting `pin=…` in the command text before hashing
//!   (`klippy/pins.py:41`). That belongs to the pin layer (TODO F2); a command
//!   added here carries its pin as it was given.
//! * **`get_query_slot`.** It places a periodic query on an absolute print-time
//!   clock (`klippy/mcu.py:1136`), which needs the clock/motion layer. The
//!   consumers that need it (ADC, endstop) bring it with them.
//! * **Sending `config_reset`.** The command is defined
//!   ([`ConfigReset`](crate::core::klippy::cmd::config::ConfigReset)) but the
//!   shutdown-recovery path that sends it is part of MCU shutdown handling
//!   (TODO B2).

use std::sync::{Mutex, MutexGuard};

use crate::core::klippy::cmd::allocate_oids::AllocateOids;
use crate::core::klippy::cmd::config::{ConfigReset, ConfigState, FinalizeConfig, GetConfig};
use crate::core::klippy::cmd::shutdown::EmergencyStop;
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{Mcu, McuError};
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use tokio::time::Duration;
use tracing::{info, warn};

/// How long a `get_config` exchange may take.
///
/// The same order as the other synchronous calls (the firmware answers from the
/// main loop, so this only has to cover a busy or half-dead link).
const CONFIG_TIMEOUT: Duration = Duration::from_secs(5);

/// The largest oid count that fits `allocate_oids count=%c`.
///
/// One byte on the wire, and the count is the number of ids handed out, so the
/// ids themselves run `0..=254`.
const MAX_OIDS: u16 = 255;

/// Which list a configuration command belongs to.
///
/// The distinction only matters when the firmware is reused: see the module
/// documentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Config,
    Restart,
    Init,
}

/// One accumulated command: a name and its arguments, in dictionary order.
#[derive(Debug, Clone, PartialEq)]
struct Command {
    name: &'static str,
    args: Vec<ArgValue>,
}

/// A callback run at build time, before the oid count is fixed.
///
/// It is handed the builder to add commands to and the MCU for the facts that
/// only exist after identify (`CLOCK_FREQ`, the dictionary). Upstream's
/// `register_config_callback` binds methods that capture both; passing them in
/// avoids the reference cycle that capturing the builder would create. It
/// returns a result because this is where a resource turns a config-file pin
/// description into a number, and an unknown or reserved pin has to fail the
/// build rather than be silently dropped.
pub type ConfigCallback = Box<dyn Fn(&ConfigBuilder, &Mcu) -> Result<(), McuError> + Send + Sync>;

/// A callback run after the firmware has accepted the configuration.
///
/// Upstream's `register_post_init_callback` (`klippy/mcu.py:1130`). It sees the
/// connected MCU so it can start periodic queries or send startup commands.
pub type PostInitCallback = Box<dyn Fn(&Mcu) + Send + Sync>;

struct State {
    /// The next id [`ConfigBuilder::create_oid`] will hand out; also the final
    /// object count.
    next_oid: u16,
    config: Vec<Command>,
    restart: Vec<Command>,
    init: Vec<Command>,
    callbacks: Vec<ConfigCallback>,
    post_init: Vec<PostInitCallback>,
    /// Move-queue slots the motion layer wants reserved (`request_move_queue_slot`).
    reserved_move_slots: u16,
    finalized: bool,
}

/// Collects the commands and object ids one MCU's configuration needs.
///
/// One builder per `[mcu]` / `[mcu <name>]` section. It is cheap to share
/// (`Arc`) and safe to call from several objects while the config file is being
/// loaded: the printer is single-threaded there, and the lock only guards
/// short, non-awaiting critical sections.
pub struct ConfigBuilder {
    state: Mutex<State>,
}

/// A built configuration: the encoded commands and the CRC that describes them.
///
/// This is the finished product of the config phase, ready to send. The
/// `post_init` callbacks are handed back so the caller decides when the firmware
/// is far enough along to run them.
pub struct BuiltConfig {
    /// CRC over the encoded `config` list, excluding `finalize_config`.
    pub crc: u32,
    /// Move-queue slots reserved during the build.
    pub move_slots: u16,
    /// `allocate_oids` + the `config` commands + `finalize_config`.
    pub config: Vec<Payload>,
    /// Commands restored on every connect.
    pub restart: Vec<Payload>,
    /// Commands sent on every connect, after the others.
    pub init: Vec<Payload>,
    post_init: Vec<PostInitCallback>,
}

impl BuiltConfig {
    /// Take the post-init callbacks, leaving none behind.
    ///
    /// Returned rather than run here because only the caller knows the firmware
    /// has accepted the configuration.
    pub fn take_post_init(&mut self) -> Vec<PostInitCallback> {
        std::mem::take(&mut self.post_init)
    }
}

impl std::fmt::Debug for BuiltConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The post-init callbacks have no useful representation.
        f.debug_struct("BuiltConfig")
            .field("crc", &self.crc)
            .field("move_slots", &self.move_slots)
            .field("config", &self.config)
            .field("restart", &self.restart)
            .field("init", &self.init)
            .finish_non_exhaustive()
    }
}

/// How a [`ConfigBuilder::configure`] went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Configured {
    /// The CRC the firmware now reports.
    pub crc: u32,
    /// The firmware's move-queue capacity.
    pub move_count: u16,
    /// True when the firmware already had this configuration and only the
    /// restart/init commands were sent.
    pub reused: bool,
}

impl ConfigBuilder {
    /// A builder with no commands, no ids, and no callbacks.
    pub fn new() -> Self {
        Self {
            state: Mutex::new(State {
                next_oid: 0,
                config: Vec::new(),
                restart: Vec::new(),
                init: Vec::new(),
                callbacks: Vec::new(),
                post_init: Vec::new(),
                reserved_move_slots: 0,
                finalized: false,
            }),
        }
    }

    /// Reserve the next object id.
    ///
    /// Ids are handed out monotonically from 0 and never reused; the firmware
    /// indexes a per-MCU table with them, and the value becomes `allocate_oids
    /// count=N` at build time. Upstream's `create_oid`
    /// (`klippy/mcu.py:1118`).
    ///
    /// # Errors
    /// Returns [`McuError::Config`] if the configuration is already built, or
    /// if [`MAX_OIDS`] ids are already out (the count and the id are one byte
    /// on the wire).
    pub fn create_oid(&self) -> Result<u8, McuError> {
        let mut state = self.lock();
        if state.finalized {
            return Err(McuError::Config(
                "cannot create an oid after the configuration is built".to_string(),
            ));
        }
        if state.next_oid >= MAX_OIDS {
            return Err(McuError::Config(format!(
                "no oids left on this MCU (limit {MAX_OIDS})"
            )));
        }
        let oid = state.next_oid as u8;
        state.next_oid += 1;
        Ok(oid)
    }

    /// Add a command to the `config` list, sent only when the firmware has to
    /// be (re)configured.
    pub fn add_config_cmd<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        self.push(Kind::Config, C::NAME, cmd.args())
    }

    /// Add a command to the `restart` list, re-sent even when the firmware is
    /// reused. Used to restore start values.
    pub fn add_restart_cmd<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        self.push(Kind::Restart, C::NAME, cmd.args())
    }

    /// Add a command to the `init` list, sent on every connect after the
    /// others. Used to arm periodic queries and startup transfers.
    pub fn add_init_cmd<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        self.push(Kind::Init, C::NAME, cmd.args())
    }

    /// Register a callback to run at build time, before the oid count is fixed.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] if the configuration is already built.
    pub fn register_config_callback(&self, callback: ConfigCallback) -> Result<(), McuError> {
        let mut state = self.lock();
        self.verify_not_finalized(&state)?;
        state.callbacks.push(callback);
        Ok(())
    }

    /// Register a callback to run once the firmware has accepted the
    /// configuration.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] if the configuration is already built.
    pub fn register_post_init_callback(&self, callback: PostInitCallback) -> Result<(), McuError> {
        let mut state = self.lock();
        self.verify_not_finalized(&state)?;
        state.post_init.push(callback);
        Ok(())
    }

    /// Reserve one slot of the firmware's move queue.
    ///
    /// Called by whatever will drive a move queue on this MCU; the build checks
    /// the firmware's `move_count` against the total. Upstream's
    /// `request_move_queue_slot` (`klippy/mcu.py:1142`).
    ///
    /// # Errors
    /// Returns [`McuError::Config`] if the configuration is already built.
    pub fn request_move_queue_slot(&self) -> Result<(), McuError> {
        let mut state = self.lock();
        self.verify_not_finalized(&state)?;
        state.reserved_move_slots += 1;
        Ok(())
    }

    /// Whether [`ConfigBuilder::build`] has run.
    pub fn is_finalized(&self) -> bool {
        self.lock().finalized
    }

    /// Run the config callbacks and freeze the configuration.
    ///
    /// Encodes the `config`, `restart` and `init` lists against `mcu`'s
    /// dictionary and computes the CRC. Call once; a second call is
    /// [`McuError::Config`]. This is upstream's `_finalize_config`
    /// (`klippy/mcu.py:1004-1020`) without the connection: the caller decides
    /// when to send the result.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] if the configuration is already built, has
    /// too many oids, or if encoding a command fails, and
    /// [`McuError::NotIdentified`] before the dictionary is installed.
    pub fn build(&self, mcu: &Mcu) -> Result<BuiltConfig, McuError> {
        // Encoding needs the firmware's dictionary, so a build before identify
        // is reported as exactly that rather than as a missing message.
        mcu.require_dictionary()?;

        // Callbacks run first, and are still allowed to create oids and add
        // commands; the lock is released so they can. A second `build` must not
        // run them again, so they are taken out and `finalized` is only set
        // after they finish — a callback that panics leaves the builder
        // buildable, which is no worse than upstream.
        {
            let mut state = self.lock();
            self.verify_not_finalized(&state)?;
            let callbacks = std::mem::take(&mut state.callbacks);
            drop(state);
            for callback in callbacks {
                callback(self, mcu)?;
            }
        }

        let (config, restart, init, post_init, oid_count, move_slots) = {
            let mut state = self.lock();
            if state.finalized {
                return Err(McuError::Config("configuration already built".to_string()));
            }
            state.finalized = true;
            (
                std::mem::take(&mut state.config),
                std::mem::take(&mut state.restart),
                std::mem::take(&mut state.init),
                std::mem::take(&mut state.post_init),
                state.next_oid,
                state.reserved_move_slots,
            )
        };
        if oid_count > MAX_OIDS {
            return Err(McuError::Config(format!(
                "too many oids for allocate_oids: {oid_count} > {MAX_OIDS}"
            )));
        }

        // `allocate_oids count=N` first, then the config commands, and the CRC
        // over exactly those bytes.
        let mut hashed: Vec<u8> = Vec::new();
        let mut config_payloads = Vec::with_capacity(config.len() + 2);
        let allocate = AllocateOids {
            count: oid_count as u8,
        };
        encode_into(
            mcu,
            AllocateOids::NAME,
            &allocate.args(),
            &mut config_payloads,
            &mut hashed,
        )?;
        for command in &config {
            encode_into(
                mcu,
                command.name,
                &command.args,
                &mut config_payloads,
                &mut hashed,
            )?;
        }
        let crc = crc32(&hashed);
        // `finalize_config` is not hashed; it carries the hash.
        let finalize = FinalizeConfig { crc };
        let payload = mcu.encode(FinalizeConfig::NAME, &finalize.args())?;
        config_payloads.push(payload);

        let encode_list = |commands: Vec<Command>| -> Result<Vec<Payload>, McuError> {
            let mut payloads = Vec::with_capacity(commands.len());
            for command in &commands {
                payloads.push(mcu.encode(command.name, &command.args)?);
            }
            Ok(payloads)
        };
        let restart = encode_list(restart)?;
        let init = encode_list(init)?;

        Ok(BuiltConfig {
            crc,
            move_slots,
            config: config_payloads,
            restart,
            init,
            post_init,
        })
    }

    /// Ask the firmware what it is configured with, then send what it needs.
    ///
    /// The connection handshake, upstream's `MCUConfigHelper._connect`
    /// (`klippy/mcu.py:1047-1085`). On success the firmware has accepted the
    /// configuration and the post-init callbacks have run.
    ///
    /// A firmware that is stopped, or that carries a different CRC, is reset
    /// first (`reset_firmware`) so that a real board can be taken over without a
    /// power cycle.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] when the firmware cannot be reset (no
    /// `config_reset`), is still stopped after a reset, refuses the
    /// configuration, or has fewer move slots than were reserved; and
    /// [`McuError::Call`] / [`McuError::Msg`] for the transport failures
    /// underneath.
    pub async fn configure(&self, mcu: &Mcu) -> Result<Configured, McuError> {
        // Building first means the CRC is known before the firmware is asked
        // anything — the reset decision below needs it.
        let BuiltConfig {
            crc,
            move_slots,
            config,
            restart,
            init,
            mut post_init,
        } = self.build(mcu)?;

        let mut before = get_config(mcu).await?;

        // A stopped firmware, or one carrying a different configuration, has to
        // be cleared before this one can be sent: `finalize_config` locks the
        // firmware (a second one shuts down with "Already finalized",
        // `src/basecmd.c:173`) and a stopped one refuses to run anything else.
        // `config_reset` clears the CRC, the oids and the shutdown latch
        // (`src/basecmd.c:262`), but only accepts to run while stopped, so an
        // emergency stop comes first when the firmware is still running.
        if before.is_shutdown || (before.is_config && before.crc != crc) {
            reset_firmware(mcu, &before, crc).await?;
            before = get_config(mcu).await?;
            if before.is_shutdown {
                return Err(McuError::Config(format!(
                    "MCU '{}' is still shutdown after config_reset",
                    mcu.name()
                )));
            }
            if before.is_config && before.crc != crc {
                return Err(McuError::Config(format!(
                    "MCU '{}' still carries CRC {:#010x} after config_reset",
                    mcu.name(),
                    before.crc
                )));
            }
        }

        let reused = before.is_config;
        if reused {
            for payload in restart {
                mcu.send_payload(payload).await?;
            }
        } else {
            for payload in config {
                mcu.send_payload(payload).await?;
            }
        }
        for payload in init {
            mcu.send_payload(payload).await?;
        }

        // The firmware answers in request order, so this second query is
        // answered after it has processed everything above.
        let after = get_config(mcu).await?;
        if !after.is_config {
            return Err(McuError::Config(format!(
                "MCU '{}' did not accept the configuration",
                mcu.name()
            )));
        }
        if after.move_count < move_slots {
            return Err(McuError::Config(format!(
                "MCU '{}' has {} move slots, {} were reserved",
                mcu.name(),
                after.move_count,
                move_slots
            )));
        }

        for callback in post_init.drain(..) {
            callback(mcu);
        }
        Ok(Configured {
            crc,
            move_count: after.move_count,
            reused,
        })
    }

    /// The final oid count, once every object has been built.
    ///
    /// Read-only view for tests and callers that want to log the configuration;
    /// it is only meaningful before [`ConfigBuilder::build`] freezes the
    /// configuration.
    pub fn oid_count(&self) -> u16 {
        self.lock().next_oid
    }

    fn push(&self, kind: Kind, name: &'static str, args: Vec<ArgValue>) -> Result<(), McuError> {
        let mut state = self.lock();
        self.verify_not_finalized(&state)?;
        let command = Command { name, args };
        match kind {
            Kind::Config => state.config.push(command),
            Kind::Restart => state.restart.push(command),
            Kind::Init => state.init.push(command),
        }
        Ok(())
    }

    fn verify_not_finalized(&self, state: &State) -> Result<(), McuError> {
        if state.finalized {
            return Err(McuError::Config(
                "the configuration is already built".to_string(),
            ));
        }
        Ok(())
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

/// Read the firmware's configuration state.
async fn get_config(mcu: &Mcu) -> Result<ConfigState, McuError> {
    mcu.call_msg::<GetConfig, ConfigState>(&GetConfig, CONFIG_TIMEOUT)
        .await
}

/// Clear a stopped or differently-configured firmware so this one can be sent.
///
/// The firmware refuses to run anything but `HF_IN_SHUTDOWN` commands once it
/// has been finalized, and `config_reset` itself only runs while stopped
/// (`src/basecmd.c:262-265`), so an emergency stop comes first when the
/// firmware is still running. Upstream does the same through its restart helper
/// (`klippy/mcu.py:756-770`), one process later; here it happens in place, which
/// is why `mcu/object.rs` binds the shutdown events only *after* this.
///
/// # Errors
/// Returns [`McuError::Config`] when the firmware has no `config_reset` (the
/// command is declared per board, not in `basecmd.c`).
async fn reset_firmware(mcu: &Mcu, state: &ConfigState, crc: u32) -> Result<(), McuError> {
    let has_reset = mcu
        .dictionary()
        .is_some_and(|dictionary| dictionary.message(ConfigReset::NAME).is_some());
    if !has_reset {
        let reason = if state.is_shutdown {
            "is shutdown".to_string()
        } else {
            format!(
                "is configured with CRC {:#010x}, the host computed {crc:#010x}",
                state.crc
            )
        };
        return Err(McuError::Config(format!(
            "MCU '{}' {reason} and the firmware has no config_reset; power-cycle \
             the board, or implement the restart path (TODO D2)",
            mcu.name()
        )));
    }

    if state.is_shutdown {
        info!(
            "MCU '{}' is shutdown; clearing its configuration",
            mcu.name()
        );
    } else {
        warn!(
            "MCU '{}' is configured with CRC {:#010x}, not {crc:#010x}; resetting it",
            mcu.name(),
            state.crc
        );
        // `config_reset` only runs while the firmware is stopped.
        mcu.send_msg(&EmergencyStop)?;
    }
    mcu.send_msg(&ConfigReset)?;
    Ok(())
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Encode a command, appending both its bytes and the payload.
fn encode_into(
    mcu: &Mcu,
    name: &str,
    args: &[ArgValue],
    payloads: &mut Vec<Payload>,
    hashed: &mut Vec<u8>,
) -> Result<(), McuError> {
    let payload = mcu.encode(name, args)?;
    hashed.extend_from_slice(payload.payload());
    payloads.push(payload);
    Ok(())
}

/// CRC-32/ISO-HDLC (the `zlib.crc32` upstream uses).
///
/// Only self-consistency matters (the firmware stores the value, it does not
/// compute it), but a standard CRC is the least surprising choice — and the
/// test below pins it to the standard check value.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use serde_json::json;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    /// A firmware dictionary with the messages the config phase needs, plus one
    /// resource command the tests add by hand.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "test_out oid=%c value=%u": 10,
                "test_restore oid=%c value=%u": 11,
                "test_init oid=%c value=%u": 12,
                "test_ping value=%u": 13,
                "config_reset": 30,
                "emergency_stop": 31
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// An identified MCU that never talks to anything.
    ///
    /// `build` only encodes, so it needs the dictionary and nothing else; the
    /// empty device is never sent to.
    fn identified_mcu() -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    /// A test config command, standing in for `config_digital_out`.
    struct TestOut {
        oid: u8,
        value: u32,
    }

    impl McuCommand for TestOut {
        const NAME: &'static str = "test_out";
        fn args(&self) -> Vec<ArgValue> {
            vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.value)]
        }
    }

    /// A test restart command.
    struct TestRestore {
        oid: u8,
        value: u32,
    }

    impl McuCommand for TestRestore {
        const NAME: &'static str = "test_restore";
        fn args(&self) -> Vec<ArgValue> {
            vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.value)]
        }
    }

    /// A test init command.
    struct TestInit {
        oid: u8,
        value: u32,
    }

    impl McuCommand for TestInit {
        const NAME: &'static str = "test_init";
        fn args(&self) -> Vec<ArgValue> {
            vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.value)]
        }
    }

    /// Decode a payload back to `(name, values)` so tests can assert on what
    /// would go on the wire.
    fn decode(mcu: &Mcu, payload: &Payload) -> (String, Vec<ArgValue>) {
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
        let decoded = parser.decode(frame.into()).unwrap();
        (decoded[0].0.name.clone(), decoded[0].1.clone())
    }

    // -----------------------------------------------------------------------
    // CRC
    // -----------------------------------------------------------------------

    #[test]
    fn test_crc32_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(b""), 0);
    }

    // -----------------------------------------------------------------------
    // oids
    // -----------------------------------------------------------------------

    #[test]
    fn test_oids_are_handed_out_from_zero_without_reuse() {
        let builder = ConfigBuilder::new();

        assert_eq!(builder.create_oid().unwrap(), 0);
        assert_eq!(builder.create_oid().unwrap(), 1);
        assert_eq!(builder.create_oid().unwrap(), 2);
        assert_eq!(builder.oid_count(), 3);
    }

    #[test]
    fn test_oid_exhaustion_is_an_error_not_a_wrap() {
        let builder = ConfigBuilder::new();
        for _ in 0..MAX_OIDS {
            builder.create_oid().unwrap();
        }

        let err = builder.create_oid().unwrap_err();
        assert!(matches!(err, McuError::Config(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_no_oid_can_be_created_after_the_build() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        builder.build(&mcu).unwrap();

        let err = builder.create_oid().unwrap_err();
        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(builder.is_finalized());
    }

    // -----------------------------------------------------------------------
    // build
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_an_empty_config_is_just_allocate_oids_and_finalize() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();

        let built = builder.build(&mcu).unwrap();

        assert_eq!(built.config.len(), 2);
        assert_eq!(built.move_slots, 0);
        let (name, args) = decode(&mcu, &built.config[0]);
        assert_eq!(name, "allocate_oids");
        assert_eq!(args, vec![ArgValue::UInt8(0)]);
        let (name, args) = decode(&mcu, &built.config[1]);
        assert_eq!(name, "finalize_config");
        assert_eq!(args, vec![ArgValue::UInt32(built.crc)]);
    }

    #[tokio::test]
    async fn test_allocate_oids_carries_the_final_count() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        let first = builder.create_oid().unwrap();
        let second = builder.create_oid().unwrap();
        builder
            .add_config_cmd(&TestOut {
                oid: second,
                value: 1,
            })
            .unwrap();
        builder
            .add_config_cmd(&TestOut {
                oid: first,
                value: 2,
            })
            .unwrap();

        let built = builder.build(&mcu).unwrap();

        // allocate_oids first, then the commands in the order they were added,
        // then finalize_config.
        assert_eq!(built.config.len(), 4);
        let (name, args) = decode(&mcu, &built.config[0]);
        assert_eq!(name, "allocate_oids");
        assert_eq!(args, vec![ArgValue::UInt8(2)]);
        let (_, args) = decode(&mcu, &built.config[1]);
        assert_eq!(args, vec![ArgValue::UInt8(1), ArgValue::UInt32(1)]);
        let (_, args) = decode(&mcu, &built.config[2]);
        assert_eq!(args, vec![ArgValue::UInt8(0), ArgValue::UInt32(2)]);
        let (name, _) = decode(&mcu, &built.config[3]);
        assert_eq!(name, "finalize_config");
    }

    #[tokio::test]
    async fn test_the_crc_is_deterministic_and_sensitive_to_the_config() {
        let mcu = identified_mcu();

        let build = |value: u32, oids: u8| -> u32 {
            let builder = ConfigBuilder::new();
            for _ in 0..oids {
                builder.create_oid().unwrap();
            }
            builder.add_config_cmd(&TestOut { oid: 0, value }).unwrap();
            builder.build(&mcu).unwrap().crc
        };

        assert_eq!(build(5, 1), build(5, 1), "same config, same CRC");
        assert_ne!(build(5, 1), build(6, 1), "a different value changes it");
        assert_ne!(build(5, 1), build(5, 2), "a different oid count changes it");
    }

    #[tokio::test]
    async fn test_restart_and_init_lists_are_separate_from_the_hashed_config() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        let oid = builder.create_oid().unwrap();
        builder.add_config_cmd(&TestOut { oid, value: 1 }).unwrap();
        builder
            .add_restart_cmd(&TestRestore { oid, value: 2 })
            .unwrap();
        builder.add_init_cmd(&TestInit { oid, value: 3 }).unwrap();

        let built = builder.build(&mcu).unwrap();

        assert_eq!(built.restart.len(), 1);
        assert_eq!(built.init.len(), 1);
        assert_eq!(decode(&mcu, &built.restart[0]).0, "test_restore");
        assert_eq!(decode(&mcu, &built.init[0]).0, "test_init");

        // The CRC covers allocate_oids + the config command only: adding a
        // restart or init command must not move it.
        let plain = ConfigBuilder::new();
        plain.create_oid().unwrap();
        plain.add_config_cmd(&TestOut { oid, value: 1 }).unwrap();
        assert_eq!(plain.build(&mcu).unwrap().crc, built.crc);
    }

    #[tokio::test]
    async fn test_config_callbacks_run_at_build_and_may_add_commands_and_oids() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        let calls = Arc::new(AtomicUsize::new(0));
        {
            let calls = Arc::clone(&calls);
            builder
                .register_config_callback(Box::new(move |builder, mcu| {
                    calls.fetch_add(1, Ordering::SeqCst);
                    // What a resource's `_build_config` does: it needs the
                    // clock, so it can only run here.
                    assert_eq!(mcu.seconds_to_clock(1.0).unwrap(), 20_000_000);
                    let oid = builder.create_oid().unwrap();
                    builder.add_config_cmd(&TestOut { oid, value: 7 }).unwrap();
                    Ok(())
                }))
                .unwrap();
        }

        let built = builder.build(&mcu).unwrap();

        assert_eq!(calls.load(Ordering::SeqCst), 1);
        let (name, args) = decode(&mcu, &built.config[0]);
        assert_eq!(name, "allocate_oids");
        assert_eq!(args, vec![ArgValue::UInt8(1)]);
        assert_eq!(decode(&mcu, &built.config[1]).0, "test_out");
    }

    #[tokio::test]
    async fn test_a_second_build_is_an_error_and_does_not_rerun_callbacks() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        let calls = Arc::new(AtomicUsize::new(0));
        {
            let calls = Arc::clone(&calls);
            builder
                .register_config_callback(Box::new(move |_, _| {
                    calls.fetch_add(1, Ordering::SeqCst);
                    Ok(())
                }))
                .unwrap();
        }
        builder.build(&mcu).unwrap();

        let err = builder.build(&mcu).unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_adding_after_the_build_is_refused() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        builder.build(&mcu).unwrap();

        assert!(builder
            .add_config_cmd(&TestOut { oid: 0, value: 0 })
            .is_err());
        assert!(builder
            .add_restart_cmd(&TestRestore { oid: 0, value: 0 })
            .is_err());
        assert!(builder
            .add_init_cmd(&TestInit { oid: 0, value: 0 })
            .is_err());
        assert!(builder.request_move_queue_slot().is_err());
        assert!(builder
            .register_config_callback(Box::new(|_, _| Ok(())))
            .is_err());
        assert!(builder
            .register_post_init_callback(Box::new(|_| {}))
            .is_err());
    }

    #[tokio::test]
    async fn test_build_before_identify_is_reported_not_guessed() {
        // A fresh transport with no dictionary: encoding cannot work, and the
        // error says so.
        let builder = ConfigBuilder::new();
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));

        let err = builder.build(&mcu).unwrap_err();

        assert!(matches!(err, McuError::NotIdentified), "{err:?}");
    }

    #[tokio::test]
    async fn test_move_queue_slots_are_counted() {
        let builder = ConfigBuilder::new();
        let mcu = identified_mcu();
        builder.request_move_queue_slot().unwrap();
        builder.request_move_queue_slot().unwrap();

        assert_eq!(builder.build(&mcu).unwrap().move_slots, 2);
    }

    #[tokio::test]
    async fn test_seconds_to_clock_uses_the_firmware_frequency() {
        let mcu = identified_mcu();

        assert_eq!(mcu.clock_freq().unwrap(), 20_000_000.0);
        assert_eq!(mcu.seconds_to_clock(0.1).unwrap(), 2_000_000);
        assert_eq!(mcu.seconds_to_clock(0.0).unwrap(), 0);
    }

    // -----------------------------------------------------------------------
    // configure — the connection handshake
    // -----------------------------------------------------------------------

    /// An identified MCU over a device scripted with `mappings`.
    fn scripted_mcu(mappings: Vec<MappingEntry>) -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(mappings)));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    /// `get_config` as the host sends it (id 7, no arguments).
    fn get_config_payload() -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(7).unwrap();
        payload.into_raw()
    }

    /// `config is_config=%c crc=%u is_shutdown=%c move_count=%hu` (id 9).
    fn config_response(is_config: bool, crc: u32, is_shutdown: bool, move_count: u16) -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(9).unwrap();
        payload.push_u8(is_config as u8).unwrap();
        payload.push_u32(crc).unwrap();
        payload.push_u8(is_shutdown as u8).unwrap();
        payload.push_u16(move_count).unwrap();
        payload.into_raw()
    }

    /// The CRC an empty configuration builds to: `allocate_oids count=0` and
    /// nothing else is hashed.
    fn empty_config_crc() -> u32 {
        let mut payload = Payload::new();
        payload.push_i16(2).unwrap(); // allocate_oids
        payload.push_u8(0).unwrap();
        crc32(payload.payload())
    }

    #[tokio::test]
    async fn test_configure_resets_a_shutdown_mcu_and_configures_it() {
        // A stopped firmware is cleared with `config_reset` (no emergency stop
        // is needed: it is already stopped) and then configured in place.
        let crc = empty_config_crc();
        let proto = identified_mcu();

        // Frame 1: `config_reset` and the follow-up `get_config`, merged by the
        // send task.
        let mut reset_frame = proto
            .encode(ConfigReset::NAME, &ConfigReset.args())
            .unwrap()
            .into_raw();
        reset_frame.extend_from_slice(&get_config_payload());

        // Frame 2: the configuration and the confirmation query.
        let shadow = ConfigBuilder::new();
        let expected = shadow.build(&proto).unwrap();
        let mut configured_frame = Vec::new();
        for payload in &expected.config {
            configured_frame.extend_from_slice(payload.payload());
        }
        configured_frame.extend_from_slice(&get_config_payload());

        let mcu = scripted_mcu(vec![
            MappingEntry {
                input: Frame::new(0, get_config_payload()),
                outputs: vec![Frame::new(0, config_response(false, 0, true, 0))],
            },
            MappingEntry {
                input: Frame::new(1, reset_frame),
                outputs: vec![Frame::new(1, config_response(false, 0, false, 100))],
            },
            MappingEntry {
                input: Frame::new(2, configured_frame),
                outputs: vec![Frame::new(2, config_response(true, crc, false, 100))],
            },
        ]);
        let builder = ConfigBuilder::new();

        let configured = builder.configure(&mcu).await.unwrap();

        assert!(!configured.reused, "the firmware was reset, not reused");
        assert_eq!(configured.crc, crc);
    }

    /// The same dictionary without `config_reset`: a firmware that cannot clear
    /// itself (the command is declared per board, not in `basecmd.c`).
    fn dictionary_without_reset() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    fn scripted_mcu_without_reset(mappings: Vec<MappingEntry>) -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(mappings)));
        mcu.install_dictionary(dictionary_without_reset()).unwrap();
        mcu
    }

    #[tokio::test]
    async fn test_configure_refuses_a_shutdown_mcu_that_cannot_reset() {
        let mcu = scripted_mcu_without_reset(vec![MappingEntry {
            input: Frame::new(0, get_config_payload()),
            outputs: vec![Frame::new(0, config_response(false, 0, true, 0))],
        }]);
        let builder = ConfigBuilder::new();

        let err = builder.configure(&mcu).await.unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(err.to_string().contains("shutdown"), "{err}");
        assert!(err.to_string().contains("config_reset"), "{err}");
    }

    #[tokio::test]
    async fn test_configure_refuses_a_different_crc_that_cannot_reset() {
        // Already configured with a CRC the host did not compute, and no way to
        // clear it: the restart path that would is TODO D2.
        let mcu = scripted_mcu_without_reset(vec![MappingEntry {
            input: Frame::new(0, get_config_payload()),
            outputs: vec![Frame::new(
                0,
                config_response(true, 0xdead_beef, false, 500),
            )],
        }]);
        let builder = ConfigBuilder::new();

        let err = builder.configure(&mcu).await.unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(err.to_string().contains("CRC"), "{err}");
    }

    #[tokio::test]
    async fn test_configure_resets_a_running_mcu_with_a_different_crc() {
        // A running firmware with another configuration: it has to be stopped
        // before `config_reset`, so the frame carries the emergency stop too.
        let crc = empty_config_crc();
        let proto = identified_mcu();

        let mut reset_frame = proto
            .encode(EmergencyStop::NAME, &EmergencyStop.args())
            .unwrap()
            .into_raw();
        reset_frame.extend_from_slice(
            &proto
                .encode(ConfigReset::NAME, &ConfigReset.args())
                .unwrap()
                .into_raw(),
        );
        reset_frame.extend_from_slice(&get_config_payload());

        let shadow = ConfigBuilder::new();
        let expected = shadow.build(&proto).unwrap();
        let mut configured_frame = Vec::new();
        for payload in &expected.config {
            configured_frame.extend_from_slice(payload.payload());
        }
        configured_frame.extend_from_slice(&get_config_payload());

        let mcu = scripted_mcu(vec![
            MappingEntry {
                input: Frame::new(0, get_config_payload()),
                outputs: vec![Frame::new(
                    0,
                    config_response(true, 0xdead_beef, false, 500),
                )],
            },
            MappingEntry {
                input: Frame::new(1, reset_frame),
                outputs: vec![Frame::new(1, config_response(false, 0, false, 100))],
            },
            MappingEntry {
                input: Frame::new(2, configured_frame),
                outputs: vec![Frame::new(2, config_response(true, crc, false, 100))],
            },
        ]);
        let builder = ConfigBuilder::new();

        let configured = builder.configure(&mcu).await.unwrap();

        assert!(!configured.reused);
        assert_eq!(configured.crc, crc);
    }

    #[tokio::test]
    async fn test_configure_reuses_a_matching_firmware_and_runs_post_init() {
        // The firmware already has exactly this (empty) configuration, so only
        // restart + init are sent — both empty — and the second query runs. No
        // configuration frame is involved, so the exchange is two single-payload
        // frames and the script is exact.
        let crc = empty_config_crc();
        let mcu = scripted_mcu(vec![
            MappingEntry {
                input: Frame::new(0, get_config_payload()),
                outputs: vec![Frame::new(0, config_response(true, crc, false, 500))],
            },
            MappingEntry {
                input: Frame::new(1, get_config_payload()),
                outputs: vec![Frame::new(1, config_response(true, crc, false, 500))],
            },
        ]);
        let builder = ConfigBuilder::new();
        let ran = Arc::new(AtomicUsize::new(0));
        {
            let ran = Arc::clone(&ran);
            builder
                .register_post_init_callback(Box::new(move |_| {
                    ran.fetch_add(1, Ordering::SeqCst);
                }))
                .unwrap();
        }

        let configured = builder.configure(&mcu).await.unwrap();

        assert!(configured.reused);
        assert_eq!(configured.crc, crc);
        assert_eq!(configured.move_count, 500);
        assert_eq!(ran.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_configure_refuses_too_few_move_slots() {
        let crc = empty_config_crc();
        let mcu = scripted_mcu(vec![
            MappingEntry {
                input: Frame::new(0, get_config_payload()),
                outputs: vec![Frame::new(0, config_response(true, crc, false, 500))],
            },
            MappingEntry {
                input: Frame::new(1, get_config_payload()),
                outputs: vec![Frame::new(1, config_response(true, crc, false, 0))],
            },
        ]);
        let builder = ConfigBuilder::new();
        builder.request_move_queue_slot().unwrap();

        let err = builder.configure(&mcu).await.unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(err.to_string().contains("move slots"), "{err}");
    }

    #[tokio::test]
    async fn test_configure_sends_a_fresh_configuration_then_confirms_it() {
        // The firmware is unconfigured, so the host sends `allocate_oids` +
        // the config commands + `finalize_config`, then asks again.
        //
        // The four payloads are queued back to back and the transport's send
        // task merges them into one frame, so the second exchange is one frame
        // carrying the whole configuration and the follow-up `get_config`.
        // A shadow builder on a throwaway MCU produces the exact bytes and CRC
        // the builder under test will produce, since both encode the same
        // commands against the same dictionary.
        let proto = identified_mcu();
        let shadow = ConfigBuilder::new();
        shadow.create_oid().unwrap();
        shadow
            .add_config_cmd(&TestOut { oid: 0, value: 7 })
            .unwrap();
        let expected = shadow.build(&proto).unwrap();

        let mut configured_frame = Vec::new();
        for payload in &expected.config {
            configured_frame.extend_from_slice(payload.payload());
        }
        configured_frame.extend_from_slice(&get_config_payload());

        let mcu = scripted_mcu(vec![
            MappingEntry {
                input: Frame::new(0, get_config_payload()),
                outputs: vec![Frame::new(0, config_response(false, 0, false, 0))],
            },
            MappingEntry {
                input: Frame::new(1, configured_frame),
                outputs: vec![Frame::new(
                    1,
                    config_response(true, expected.crc, false, 500),
                )],
            },
        ]);
        let builder = ConfigBuilder::new();
        builder
            .add_config_cmd(&TestOut {
                oid: builder.create_oid().unwrap(),
                value: 7,
            })
            .unwrap();

        let configured = builder.configure(&mcu).await.unwrap();

        assert!(!configured.reused);
        assert_eq!(configured.crc, expected.crc);
        assert_eq!(configured.move_count, 500);
        assert!(builder.is_finalized());
    }
}
