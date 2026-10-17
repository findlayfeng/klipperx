//! Bench diagnostic: read the bridge firmware's own state over its console.
//!
//! A side channel for asking a *running* bridge what it thinks — port flags and
//! routes, the CAN backend's counters, the hardware receive filters, a node's
//! reassembly buffer — without a debugger. It uses the firmware's `debug_read`
//! (`src/debugcmds.c`, `HF_IN_SHUTDOWN`) through the bridge's own mux console,
//! so nothing has to be unplugged or re-flashed.
//!
//! The **addresses are not in the firmware's dictionary**: get them from the
//! ELF the board is running, e.g.
//!
//! ```text
//! readelf -sW target/hw-can-bridge/out4/klipper.elf | grep bridge_can
//! arm-none-eabi-gdb -batch -ex "ptype /o bridge_can" <elf>   # field offsets
//! ```
//!
//! Usage:
//!
//! ```text
//! cargo run --example bridge-read -- <bridge tty> <bridge uuid hex> \
//!     port=4:0x200008dc node_id=1:0x200008e0 rx_ignored=4:0x20000af4
//! ```
//!
//! Each entry is `<label>=<size>:<address>` with size 1, 2 (16-bit) or 4 (32-bit);
//! addresses are hex and, on a 32-bit MCU, used as-is (`command_decode_ptr`).

use std::path::Path;
use std::time::Duration;

use klipperx::core::klippy::msg::proto::ArgValue;
use klipperx::flash::bridge_flash::{BridgeLink, BRIDGE_MCU_ID};

/// One `label=size:addr` request.
struct Read {
    label: String,
    /// `debug_read`'s access size: 0 = 8-bit, 1 = 16-bit, 2 = 32-bit.
    order: u8,
    addr: u32,
}

fn parse_uuid(text: &str) -> [u8; 6] {
    let raw: Vec<u8> = (0..text.len() / 2)
        .map(|i| u8::from_str_radix(&text[i * 2..i * 2 + 2], 16).expect("hex"))
        .collect();
    raw.try_into().expect("twelve hex digits")
}

fn parse_read(spec: &str) -> Result<Read, String> {
    let (label, rest) = spec
        .split_once('=')
        .ok_or_else(|| format!("'{spec}' is not <label>=<size>:<address>"))?;
    let (size, addr) = rest
        .split_once(':')
        .ok_or_else(|| format!("'{spec}' is not <label>=<size>:<address>"))?;
    let order = match size {
        "1" => 0,
        "2" => 1,
        "4" => 2,
        other => return Err(format!("size '{other}' is not 1, 2 or 4")),
    };
    let addr = u32::from_str_radix(addr.trim_start_matches("0x"), 16)
        .map_err(|e| format!("address '{addr}': {e}"))?;
    Ok(Read {
        label: label.to_string(),
        order,
        addr,
    })
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or("usage: bridge-read <bridge tty> <bridge uuid hex> [label=size:addr ...]")?;
    let uuid = parse_uuid(&args.next().ok_or("missing bridge uuid")?);
    let reads: Vec<Read> = args
        .map(|spec| parse_read(&spec))
        .collect::<Result<_, _>>()?;
    if reads.is_empty() {
        return Err("give at least one label=size:addr (see this file's docs)".into());
    }

    let link = BridgeLink::open_serial(Path::new(&path), 250_000, BRIDGE_MCU_ID, Some(uuid))?;
    link.identify(Duration::from_secs(10)).await?;
    let mcu = link.mcu().ok_or("no live mcu after identify")?;

    for read in &reads {
        let reply = mcu
            .call(
                "debug_read",
                &[ArgValue::UInt8(read.order), ArgValue::UInt32(read.addr)],
                "debug_result",
                Duration::from_secs(2),
            )
            .await;
        match reply {
            Ok(values) => match values.first() {
                Some(ArgValue::UInt32(v)) => {
                    println!("{:28} {:#06x} ({})", read.label, v, v);
                }
                other => println!("{:28} <unexpected reply {other:?}>", read.label),
            },
            Err(err) => println!("{:28} <{err}>", read.label),
        }
    }
    Ok(())
}
