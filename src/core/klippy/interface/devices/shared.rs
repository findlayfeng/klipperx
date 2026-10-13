//! The reuse spine the two **shared** transports stand on: one resource per
//! path, opened once, referenced by several handles, torn down when the last
//! reference is dropped.
//!
//! Two device modules build on `resource_for`:
//!
//! * [`serial`](crate::core::klippy::interface::devices::serial) — a serial
//!   mux: one tty opened once, several MCUs on it
//!   ([`SharedSerialPort`](crate::core::klippy::interface::devices::serial::SharedSerialPort) /
//!   [`SharedSerialHandle`](crate::core::klippy::interface::devices::serial::SharedSerialHandle)),
//!   routed by an arbitration header.
//! * [`host`](crate::core::klippy::interface::devices::host) — a klipper host
//!   library loaded once, several communication references on it
//!   (`HostLibrary` /
//!   [`HostDevice`](crate::core::klippy::interface::devices::host::HostDevice)).
//!
//! Each owns a process-wide registry (`HashMap<PathBuf, Weak<Resource>>`,
//! lazily initialised) and hands it to `resource_for` together with its open
//! function: the first reference opens the resource and registers it, later
//! references on the same path share it, and the resource is torn down only
//! when its **last** reference is gone — at which point the `Weak` entry is
//! dead and the next lookup opens a fresh one. `shutdown` on a reference stops
//! only that reference, never the shared resource.
//!
//! The wire logic of each transport is deliberately **not** here — a generic
//! registry has nothing to say about headers or firmware; it only holds the one
//! shape both transports happen to share.

use crate::core::klippy::interface::error::InterfaceError;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, Weak};

/// The reuse spine both shared transports stand on.
///
/// Look `path` up in `registry`; if no live resource is registered for it,
/// open one with `open` and register a [`Weak`] entry. The caller holds the
/// returned `Arc`, so the resource lives exactly as long as its references do;
/// when the last one is gone the `Weak` entry is dead and the next lookup
/// opens a fresh resource.
pub(crate) fn resource_for<T>(
    registry: &Mutex<HashMap<PathBuf, Weak<T>>>,
    path: &Path,
    open: impl FnOnce(&Path) -> Result<Arc<T>, InterfaceError>,
) -> Result<Arc<T>, InterfaceError> {
    let mut registry = registry.lock().unwrap();
    if let Some(existing) = registry.get(path).and_then(Weak::upgrade) {
        return Ok(existing);
    }
    // A dead weak entry (its resource was dropped) is replaced by the fresh one.
    let resource = open(path)?;
    registry.insert(path.to_path_buf(), Arc::downgrade(&resource));
    Ok(resource)
}
