//! The MCU as an SPI bus: `McuSpi`, upstream's `MCU_SPI` (`klippy/extras/bus.py:42`).
//!
//! A device is created with `config_spi` (which also installs the chip-select
//! pin the firmware drives) or `config_spi_without_cs`, then configured as
//! either a **hardware bus** (`spi_set_bus`, using the MCU's SPI peripheral) or
//! a **software bus** (`spi_set_sw_bus`, bit-banging on GPIO pins). After that,
//! bytes move with `spi_send` (write only) or `spi_transfer` (full duplex);
//! `transfer_with_preface` sends a preface and the read that follows it as one
//! block, upstream's two-message shape.
//!
//! # Chip select
//!
//! The firmware owns the CS pin: every transfer asserts it, shifts the bytes,
//! and releases it. That is why the resource does not drive it itself, and why
//! a read that needs the device held across several commands is expressed as one
//! `spi_transfer` (a command byte plus the dummy bytes it clocks out).
//!
//! # Lifecycle
//!
//! 1. **Build time** (`McuSpi::new`): register the config callback. The mode
//!    (hardware vs software) and CS pin are decided here.
//! 2. **Config callback**: resolve the CS and bus/pin numbers, add
//!    `config_spi` / `spi_set_bus` / `spi_set_sw_bus`.
//! 3. **Runtime**: `transfer()` shifts bytes and returns what came back;
//!    `send()` shifts without waiting.
//! 4. **Shutdown**: a message registered with `setup_shutdown_msg` before the
//!    config is built becomes `config_spi_shutdown`, which the firmware keeps
//!    and clocks out on its own when it shuts down.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::core::klippy::cmd::spi::{
    add_software_bus, ConfigSpi, ConfigSpiShutdown, ConfigSpiWithoutCs, SoftwareSpiBus, SpiSend,
    SpiSetBus, SpiTransfer, SpiTransferResponse,
};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::pins::{PinParams, PrinterPins};

use super::pin::pin_number;

/// How long a `spi_transfer` may take before it is reported as a timeout.
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(1);

/// The mode an SPI device uses to talk to the bus.
#[derive(Debug, Clone)]
pub enum SpiMode {
    /// Hardware SPI peripheral: `spi_set_bus` with a bus name.
    Hardware {
        /// Bus enumeration name (e.g. `"spi1a"`), or `None` for the bus named
        /// `0`.
        bus: Option<String>,
        /// Clock frequency in Hz.
        speed: u32,
        /// SPI mode (CPOL/CPHA), 0..=3.
        mode: u8,
    },
    /// Software (bit-banged) SPI on GPIO pins: `spi_set_sw_bus`.
    ///
    /// The pins are **names** as written in the config (aliases already
    /// resolved by the section); they become numbers in the config callback,
    /// when the firmware dictionary can be asked.
    Software {
        /// MISO pin name.
        miso_pin: String,
        /// MOSI pin name.
        mosi_pin: String,
        /// SCLK pin name.
        sclk_pin: String,
        /// Clock frequency in Hz (used to compute `pulse_ticks`).
        speed: u32,
        /// SPI mode (CPOL/CPHA), 0..=3.
        mode: u8,
    },
}

/// The state a config callback captures and runtime methods read.
struct SpiState {
    /// The oid the firmware assigned.
    oid: Mutex<Option<u8>>,
    /// Which mode was configured (hardware vs software).
    mode: Mutex<SpiMode>,
    /// The chip-select pin, or `None` for `config_spi_without_cs`.
    cs_pin: Mutex<Option<PinParams>>,
    /// Whether the CS pin is active high.
    cs_active_high: Mutex<bool>,
    /// Messages registered by `setup_shutdown_msg`, each with the shutdown
    /// handler's oid already allocated. `build` turns them into
    /// `config_spi_shutdown`, which is where the device's oid is known.
    shutdown: Mutex<Vec<(u8, Vec<u8>)>>,
}

/// One SPI device on an MCU.
pub struct McuSpi {
    state: Arc<SpiState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
    /// The chip's config builder: `setup_shutdown_msg` allocates its handler
    /// oid here, before the config is built.
    config: Arc<ConfigBuilder>,
}

impl McuSpi {
    /// Build the resource and register its config callback.
    ///
    /// # Panics
    /// A resource is always built while the config file is loaded, before the
    /// configuration is frozen. The config callback runs later, at build time.
    pub fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: &str,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        mode: SpiMode,
        cs_pin: Option<PinParams>,
        cs_active_high: bool,
    ) -> Self {
        let state = Arc::new(SpiState {
            oid: Mutex::new(None),
            mode: Mutex::new(mode),
            cs_pin: Mutex::new(cs_pin),
            cs_active_high: Mutex::new(cs_active_high),
            shutdown: Mutex::new(Vec::new()),
        });

        let callback_state = Arc::clone(&state);
        let callback_pins = Arc::downgrade(&pins);
        let callback_chip_name = chip_name.to_string();

        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(builder, mcu, &pins, &callback_chip_name)
            }))
            .expect("a resource is always built before the configuration is");

        Self { state, mcu, config }
    }

    /// The oid the firmware allocated.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    pub(crate) fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .expect("oid lock poisoned")
            .ok_or_else(|| McuError::Config("SPI device is not configured yet".to_string()))
    }

    /// The connected MCU, or an error if not connected.
    pub(crate) fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu
            .lock()
            .expect("mcu lock poisoned")
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))
    }

    /// Shift `data` out and return the bytes clocked in, the same length.
    ///
    /// A single call is one chip-select pulse, so a device protocol that needs
    /// the select held across a command and its data is expressed here as one
    /// buffer (e.g. `9f 00 00 00` for a JEDEC-ID read).
    ///
    /// # Errors
    /// Returns [`McuError::Call`] on a send failure or timeout, and
    /// [`McuError::Decode`] when the response cannot be decoded.
    pub async fn transfer(&self, data: &[u8]) -> Result<Vec<u8>, McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;

        let command = SpiTransfer {
            oid,
            data: data.to_vec(),
        };
        let response: SpiTransferResponse = tokio::time::timeout(
            TRANSFER_TIMEOUT,
            mcu.call_msg_oid(&command, oid, TRANSFER_TIMEOUT),
        )
        .await
        .map_err(|_| {
            McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                "spi_transfer timed out".to_string(),
            ))
        })??;
        Ok(response.response)
    }

    /// Shift `preface` out, then read `data` back, as upstream's
    /// `spi_transfer_with_preface`: two messages in **one** block, one
    /// acknowledgement.
    ///
    /// The block is `spi_send`(preface) followed by `spi_transfer`(zeros for the
    /// read), which is exactly what upstream encodes into one send
    /// (`klippy/extras/bus.py:117-122`: the preface is `spi_send_cmd`, the read
    /// is `spi_transfer_cmd`, both handed to `send_with_preface`,
    /// `klippy/mcu.py:104-111`). The firmware still runs each message on its
    /// own — two `spidev_transfer` calls, so **two chip-select pulses**
    /// (`src/spicmds.c:113-132`) — but they are adjacent on the wire with no
    /// host round trip and no other message between them, and the pair costs
    /// **one** ack instead of two.
    ///
    /// The preface must be a write the device does not answer: `spi_send` emits
    /// no response (`src/spicmds.c:125-132`), so the only answer the block
    /// produces is the read's. That is why the preface is not a second
    /// `spi_transfer` — one of those always answers, and the preface's echo
    /// would be the answer this call decoded.
    ///
    /// The read segment clocks zeros out, and the answer lands in `data` (it is
    /// `data.len()` bytes). An empty preface sends an empty `spi_send`, which is
    /// what upstream does too: a caller with no preface uses
    /// [`McuSpi::transfer`].
    ///
    /// # Errors
    /// What [`McuSpi::transfer`] reports, plus [`McuError::Decode`] when the
    /// answer is not `data.len()` bytes long, and [`McuError::Msg`] when the two
    /// messages together exceed one wire block (see [`Mcu::call_two`]).
    pub async fn transfer_with_preface(
        &self,
        preface: &[u8],
        data: &mut [u8],
    ) -> Result<(), McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;

        let response: SpiTransferResponse = tokio::time::timeout(
            TRANSFER_TIMEOUT,
            mcu.call_two(
                &SpiSend {
                    oid,
                    data: preface.to_vec(),
                },
                &SpiTransfer {
                    oid,
                    data: vec![0; data.len()],
                },
                oid,
                TRANSFER_TIMEOUT,
            ),
        )
        .await
        .map_err(|_| {
            McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                "spi_transfer timed out".to_string(),
            ))
        })??;

        if response.response.len() != data.len() {
            return Err(McuError::Decode(format!(
                "spi_transfer answered {} bytes for a {} byte read",
                response.response.len(),
                data.len()
            )));
        }
        data.copy_from_slice(&response.response);
        Ok(())
    }

    /// Register a message the firmware clocks out on this device's chip select
    /// when it shuts down: `config_spi_shutdown`.
    ///
    /// Upstream's `MCU_SPI.setup_shutdown_msg` (`klippy/extras/bus.py:74-78`):
    /// the handler gets an oid of its own, and the command names both it and
    /// this device. The firmware keeps the message in its shutdown table and
    /// shifts it out itself (`src/spicmds.c:145-179`), so a driver can disable
    /// its hardware without racing the host for the bus.
    ///
    /// The message belongs to the config package, so this has to run while the
    /// configuration is still being built. Like upstream, each call registers
    /// another message (a new handler oid each time); the firmware sends them
    /// in the order they were registered.
    ///
    /// # Errors
    /// [`McuError::Config`] when the configuration is already built (no oid is
    /// left to allocate, so the message could never reach the firmware).
    pub fn setup_shutdown_msg(&self, msg: &[u8]) -> Result<(), McuError> {
        let oid = self
            .config
            .create_oid()
            .map_err(|e| McuError::Config(format!("config_spi_shutdown: {e}")))?;
        self.state
            .shutdown
            .lock()
            .expect("shutdown lock poisoned")
            .push((oid, msg.to_vec()));
        Ok(())
    }

    /// Shift `data` out, ignoring what comes back.
    ///
    /// The firmware sends no response, so this only queues the command. It is
    /// the `spi_send` fast path (a write that needs no read).
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected or the send buffer is
    /// full.
    pub fn send(&self, data: &[u8]) -> Result<(), McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;
        mcu.send_msg(&SpiSend {
            oid,
            data: data.to_vec(),
        })
    }

    /// The chip-select pin, or `None` when the device has none.
    pub fn cs_pin(&self) -> Option<PinParams> {
        self.state
            .cs_pin
            .lock()
            .expect("cs pin lock poisoned")
            .clone()
    }
}

impl SpiState {
    /// Build the device's config commands.
    ///
    /// This runs in the config callback, the first moment the dictionary exists.
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
    ) -> Result<(), McuError> {
        // Allocate an oid — the firmware assigns it.
        let oid = builder.create_oid()?;
        *self.oid.lock().expect("oid lock poisoned") = Some(oid);

        // Chip select: the firmware drives it around every transfer.
        let cs_pin = self.cs_pin.lock().expect("cs pin lock poisoned").clone();
        let cs_active_high = *self.cs_active_high.lock().expect("cs lock poisoned");
        match cs_pin {
            Some(params) => {
                let pin = pins
                    .resolve_pin(chip_name, &params.pin)
                    .map_err(|e| McuError::Config(format!("spi cs pin: {e}")))?;
                let pin = pin_number(mcu, &pin, chip_name)
                    .map_err(|e| McuError::Config(format!("spi cs pin: {e}")))?;
                builder
                    .add_config_cmd(&ConfigSpi {
                        oid,
                        pin,
                        cs_active_high,
                    })
                    .map_err(|e| McuError::Config(format!("config_spi: {e}")))?;
            }
            None => {
                builder
                    .add_config_cmd(&ConfigSpiWithoutCs { oid })
                    .map_err(|e| McuError::Config(format!("config_spi_without_cs: {e}")))?;
            }
        }

        // Add the bus config command based on mode.
        let mode = self.mode.lock().expect("mode lock poisoned").clone();
        match mode {
            SpiMode::Hardware {
                ref bus,
                speed,
                mode,
            } => {
                // The firmware declares `spi_bus=%u`, so the name the config
                // wrote has to become the enumeration's value here: this port
                // encodes config commands instead of sending them as text, so
                // the firmware never sees the name to resolve.
                let bus = pins
                    .resolve_bus_value(mcu, "spi_bus", bus.as_deref())
                    .map_err(|e| McuError::Config(format!("spi bus: {e}")))?;
                builder
                    .add_config_cmd(&SpiSetBus {
                        oid,
                        bus,
                        mode,
                        rate: speed,
                    })
                    .map_err(|e| McuError::Config(format!("spi_set_bus: {e}")))?;
            }
            SpiMode::Software {
                ref miso_pin,
                ref mosi_pin,
                ref sclk_pin,
                speed,
                mode,
            } => {
                // The pins arrive as names; the firmware wants numbers, which
                // only exist once the dictionary does. Aliases resolve first,
                // exactly as for a digital output.
                let miso = pins
                    .resolve_pin(chip_name, miso_pin)
                    .map_err(|e| McuError::Config(format!("spi miso pin: {e}")))?;
                let miso = pin_number(mcu, &miso, chip_name)
                    .map_err(|e| McuError::Config(format!("spi miso pin: {e}")))?;
                let mosi = pins
                    .resolve_pin(chip_name, mosi_pin)
                    .map_err(|e| McuError::Config(format!("spi mosi pin: {e}")))?;
                let mosi = pin_number(mcu, &mosi, chip_name)
                    .map_err(|e| McuError::Config(format!("spi mosi pin: {e}")))?;
                let sclk = pins
                    .resolve_pin(chip_name, sclk_pin)
                    .map_err(|e| McuError::Config(format!("spi sclk pin: {e}")))?;
                let sclk = pin_number(mcu, &sclk, chip_name)
                    .map_err(|e| McuError::Config(format!("spi sclk pin: {e}")))?;

                // Which command that is — and whether the tick count or the
                // raw rate goes out — is the command layer's business: newer
                // firmware gets `spi_set_sw_bus`, older gets
                // `spi_set_software_bus`.
                add_software_bus(
                    builder,
                    mcu,
                    &SoftwareSpiBus {
                        oid,
                        miso_pin: miso,
                        mosi_pin: mosi,
                        sclk_pin: sclk,
                        mode,
                        speed,
                    },
                )?;
            }
        }

        // Shutdown messages registered with `setup_shutdown_msg`: their handler
        // oid was allocated when they were registered, and this device's oid
        // only exists now.
        let shutdown = std::mem::take(&mut *self.shutdown.lock().expect("shutdown lock poisoned"));
        for (shutdown_oid, shutdown_msg) in shutdown {
            builder
                .add_config_cmd(&ConfigSpiShutdown {
                    oid: shutdown_oid,
                    spi_oid: oid,
                    shutdown_msg,
                })
                .map_err(|e| McuError::Config(format!("config_spi_shutdown: {e}")))?;
        }

        Ok(())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::devices::frame_mock::RecordingWire;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, McuChip};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use serde_json::json;
    use std::time::Duration;

    /// The `spi_send` message id in [`dictionary`], pinned in the wire
    /// assertions below.
    const SPI_SEND_ID: i16 = 54;
    /// The `spi_transfer` message id in [`dictionary`].
    const SPI_TRANSFER_ID: i16 = 55;
    /// The `spi_transfer_response` message id in [`dictionary`].
    const SPI_TRANSFER_RESPONSE_ID: i16 = 56;

    /// The messages these tests move: a hardware bus, one device without a
    /// chip select, and the shutdown registration.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_spi_without_cs oid=%c": 51,
                "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u": 52,
                "spi_send oid=%c data=%*s": 54,
                "spi_transfer oid=%c data=%*s": SPI_TRANSFER_ID,
                "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s": 60
            },
            "responses": {
                "spi_transfer_response oid=%c response=%*s": SPI_TRANSFER_RESPONSE_ID,
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {"spi_bus": {"spi1a": 0}},
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// Closes the fake session when a test ends — on a failed assertion too.
    ///
    /// The parked device read runs inside `spawn_blocking`; a session left open
    /// would hold up the runtime's shutdown.
    struct SessionGuard(Arc<Mcu>);

    impl Drop for SessionGuard {
        fn drop(&mut self) {
            self.0.clear_events();
            self.0.close();
        }
    }

    /// One `McuSpi` on a recording wire, with the chip and the builder a test
    /// needs to run the config callback.
    struct Harness {
        /// Dropped first, so the session closes before the rest goes.
        _session: SessionGuard,
        device: Arc<McuSpi>,
        builder: Arc<ConfigBuilder>,
        mcu: Arc<Mcu>,
        wire: RecordingWire,
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
        /// The registry the resource holds a weak handle to; kept alive here.
        _pins: Arc<PrinterPins>,
    }

    fn harness() -> Harness {
        let wire = RecordingWire::new();
        let handle = wire.clone();
        let sent = wire.sent();
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu.set_clock_base(0);
        let session = SessionGuard(Arc::clone(&mcu));

        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        chip.attach(Arc::clone(&mcu));
        let device = chip.setup_spi(
            SpiMode::Hardware {
                bus: Some("spi1a".to_string()),
                speed: 1_000_000,
                mode: 0,
            },
            None,
            false,
        );
        Harness {
            _session: session,
            device,
            builder: chip.config(),
            mcu,
            wire: handle,
            sent,
            _pins: pins,
        }
    }

    /// One message's bytes, as the firmware reads them.
    fn wire_bytes(format: &str, id: i16, args: &[ArgValue]) -> Vec<u8> {
        let mut parser = Parser::new();
        parser.register(id, format).unwrap();
        let name = format.split_whitespace().next().unwrap();
        parser.encode(name, args).unwrap().into_raw()
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|byte| format!("{byte:02x}")).collect()
    }

    /// The payloads the wire took so far, oldest first.
    fn sends(sent: &Arc<Mutex<Vec<Vec<u8>>>>) -> Vec<Vec<u8>> {
        sent.lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// The block `transfer_with_preface(preface, read_len)` puts on the wire:
    /// `spi_send`(preface) immediately followed by `spi_transfer`(zeros).
    fn preface_block(oid: u8, preface: &[u8], read_len: usize) -> Vec<u8> {
        let mut block = wire_bytes(
            "spi_send oid=%c data=%*s",
            SPI_SEND_ID,
            &[ArgValue::UInt8(oid), ArgValue::Bytes(preface.to_vec())],
        );
        block.extend(wire_bytes(
            "spi_transfer oid=%c data=%*s",
            SPI_TRANSFER_ID,
            &[ArgValue::UInt8(oid), ArgValue::Bytes(vec![0; read_len])],
        ));
        block
    }

    /// Script the answer to `request`: one `spi_transfer_response`, because the
    /// block's other message is a `spi_send` and a `spi_send` answers nothing
    /// (`src/spicmds.c:125-132`).
    fn answer(wire: &RecordingWire, oid: u8, request: Vec<u8>, response: &[u8]) {
        wire.reply_to(
            request,
            wire_bytes(
                "spi_transfer_response oid=%c response=%*s",
                SPI_TRANSFER_RESPONSE_ID,
                &[ArgValue::UInt8(oid), ArgValue::Bytes(response.to_vec())],
            ),
        );
    }

    /// `transfer_with_preface` is upstream's shape (`bus.py:117-122`): the
    /// preface and the read are two messages in **one** block — one ack, one
    /// round trip — and the firmware still runs each message on its own, so the
    /// pair is two chip-select pulses.
    #[tokio::test]
    async fn test_transfer_with_preface_is_two_messages_in_one_block() {
        let harness = harness();
        harness.builder.build(&harness.mcu).unwrap();
        let oid = harness.device.oid().unwrap();
        let request = preface_block(oid, &[0x9f], 3);
        // A W25's answer to a JEDEC-ID read: `ef 40 17`. The preface is a
        // `spi_send`, so it echoes nothing into this answer.
        answer(&harness.wire, oid, request.clone(), &[0xef, 0x40, 0x17]);

        let mut data = [0u8; 3];
        tokio::time::timeout(
            Duration::from_secs(5),
            harness.device.transfer_with_preface(&[0x9f], &mut data),
        )
        .await
        .expect("the transfer is bounded")
        .expect("the canned answer arrives");

        assert_eq!(data, [0xef, 0x40, 0x17]);

        let sends = sends(&harness.sent);
        assert_eq!(
            sends.len(),
            1,
            "one block: one acknowledgement, no second round trip"
        );
        // `36` = `spi_send` (id 54) with the preface, then `37` = `spi_transfer`
        // (id 55) with the three bytes the read clocks out: both message
        // headers are in one frame.
        assert_eq!(hex(&sends[0]), "3600019f370003000000");
        assert_eq!(sends[0], request, "the block is exactly what was scripted");

        let mut parser = Parser::new();
        harness
            .mcu
            .dictionary()
            .unwrap()
            .install(&mut parser)
            .unwrap();
        let decoded = parser.decode(Payload::from_raw(sends[0].clone())).unwrap();
        assert_eq!(
            decoded
                .iter()
                .map(|(msg, _)| msg.name.as_str())
                .collect::<Vec<_>>(),
            ["spi_send", "spi_transfer"],
            "two messages, run by the firmware as two `spidev_transfer` calls \
             (`src/spicmds.c:113-132`): one chip-select pulse each"
        );
        assert_eq!(
            decoded[0].1,
            vec![ArgValue::UInt8(oid), ArgValue::Bytes(vec![0x9f])]
        );
        assert_eq!(
            decoded[1].1,
            vec![
                ArgValue::UInt8(oid),
                ArgValue::Bytes(vec![0x00, 0x00, 0x00])
            ]
        );
        assert_eq!(
            harness.mcu.pending_call_count().await,
            0,
            "the answer took the one registration with it"
        );
    }

    /// ② `setup_shutdown_msg` reaches the config package as
    /// `config_spi_shutdown`, one message per call, each with its own handler
    /// oid and this device's oid; the bytes are pinned.
    #[tokio::test]
    async fn test_setup_shutdown_msg_is_one_config_command_per_call() {
        let harness = harness();
        harness.device.setup_shutdown_msg(&[0x80, 0xff]).unwrap();
        // Upstream allocates a new handler oid per call (`bus.py:74-78`), so a
        // second message is a second config command, not a replacement.
        harness.device.setup_shutdown_msg(&[0x00]).unwrap();
        let built = harness.builder.build(&harness.mcu).unwrap();
        let oid = harness.device.oid().unwrap();

        let mut parser = Parser::new();
        harness
            .mcu
            .dictionary()
            .unwrap()
            .install(&mut parser)
            .unwrap();
        let mut shutdown = Vec::new();
        for payload in &built.config {
            let bytes = payload.payload().to_vec();
            let decoded = parser.decode(Payload::from_raw(bytes.clone())).unwrap();
            if decoded[0].0.name == "config_spi_shutdown" {
                shutdown.push((hex(&bytes), decoded[0].1.clone()));
            }
        }

        assert_eq!(shutdown.len(), 2, "one config command per call");
        // `3c` = message id 60, then the handler oid, this device's oid, the
        // `%*s` length and the message itself. The handler oids are allocated
        // by `setup_shutdown_msg`, so they precede the device's own oid.
        assert_eq!(oid, 2, "the device's oid is allocated at build time");
        assert_eq!(shutdown[0].0, "3c00020280ff");
        assert_eq!(
            shutdown[0].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt8(oid),
                ArgValue::Bytes(vec![0x80, 0xff]),
            ]
        );
        assert_eq!(shutdown[1].0, "3c01020100");
        assert_eq!(
            shutdown[1].1,
            vec![
                ArgValue::UInt8(1),
                ArgValue::UInt8(oid),
                ArgValue::Bytes(vec![0x00]),
            ]
        );
    }

    /// The message belongs to the config package: after the build there is no
    /// oid left to allocate, so the call is refused rather than silently lost.
    #[tokio::test]
    async fn test_setup_shutdown_msg_after_the_build_is_refused() {
        let harness = harness();
        harness.builder.build(&harness.mcu).unwrap();

        let err = harness.device.setup_shutdown_msg(&[0x80]).unwrap_err();

        assert!(
            err.to_string()
                .contains("cannot create an oid after the configuration is built"),
            "{err}"
        );
    }
}
