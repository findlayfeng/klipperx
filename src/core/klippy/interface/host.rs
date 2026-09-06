// Dynamic library interface for KlippyInterface
//
// This module provides a real implementation of KlippyInterface using
// the klipper host library (libklipper_host.so). The library is loaded
// dynamically at runtime via libloading (dlopen/dlsym).
//
// The library path must be provided during initialization.
// No default path, no environment variable, no fallback.
// If no path is provided, initialization will fail.
//
// The Klipper host library provides a thread-safe I/O API:
//   klipper_host_init()     - Initialize the library
//   klipper_host_input()    - Feed input data (klipper binary protocol)
//   klipper_host_output()   - Retrieve output data (klipper message format)
//   klipper_host_run()      - Run the main scheduling loop (blocking)
//   klipper_host_step()     - Run a single iteration (non-blocking)
//   klipper_host_shutdown() - Request shutdown (thread-safe)
//   klipper_host_is_shutdown()- Check shutdown status
//   klipper_host_get_clock() - Get current clock time

#![allow(static_mut_refs)]

use crate::core::klippy::error::KlippyError;
use crate::core::klippy::traits::{InterfaceEvent, KlippyInterface};
use tracing::{info, warn};

use libloading::{Library, Symbol};
use std::collections::HashMap;
use std::os::raw::{c_uchar, c_long};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

use crate::core::klippy::Payload;

/// Handler callback type
type EventHandler = Arc<dyn Fn(&[u8]) + Send + Sync>;

// ============================================================
// Klipper host library function pointer types
// ============================================================

type KlipperHostInitFn = unsafe extern "C" fn() -> c_long;
type KlipperHostInputFn = unsafe extern "C" fn(*const c_uchar, usize) -> c_long;
type KlipperHostOutputFn = unsafe extern "C" fn(*mut c_uchar, usize) -> usize;
type KlipperHostRunFn = unsafe extern "C" fn();
type KlipperHostStepFn = unsafe extern "C" fn();
type KlipperHostShutdownFn = unsafe extern "C" fn();
type KlipperHostIsShutdownFn = unsafe extern "C" fn() -> c_uchar;
type KlipperHostGetClockFn = unsafe extern "C" fn() -> u32;

// ============================================================
// Dynamic library loader
// ============================================================

/// Global library handle and function pointers
static mut KLIPPER_LIB: Option<Arc<Library>> = None;
static mut KLIPPER_INIT_FN: Option<Symbol<'static, KlipperHostInitFn>> = None;
static mut KLIPPER_INPUT_FN: Option<Symbol<'static, KlipperHostInputFn>> = None;
static mut KLIPPER_OUTPUT_FN: Option<Symbol<'static, KlipperHostOutputFn>> = None;
static mut KLIPPER_RUN_FN: Option<Symbol<'static, KlipperHostRunFn>> = None;
static mut KLIPPER_STEP_FN: Option<Symbol<'static, KlipperHostStepFn>> = None;
static mut KLIPPER_SHUTDOWN_FN: Option<Symbol<'static, KlipperHostShutdownFn>> = None;
static mut KLIPPER_IS_SHUTDOWN_FN: Option<Symbol<'static, KlipperHostIsShutdownFn>> = None;
static mut KLIPPER_GET_CLOCK_FN: Option<Symbol<'static, KlipperHostGetClockFn>> = None;

/// Global initialization flag
static mut KLIPPER_INITIALIZED: bool = false;
static KLIPPER_INIT: std::sync::Once = std::sync::Once::new();

/// Load the klipper host shared library dynamically from the given path.
fn load_klipper_library(lib_path: &std::path::Path) -> Result<(), KlippyError> {
    unsafe {
        // Check if already loaded
        if KLIPPER_LIB.is_some() {
            return Ok(());
        }

        // Load the shared library
        let lib = Library::new(lib_path).map_err(|e| {
            KlippyError::Internal(format!(
                "Failed to load libklipper_host.so: {} (path: {})",
                e,
                lib_path.display()
            ))
        })?;

        info!("[KLIPPER] Library loaded from: {}", lib_path.display());

        // Store the library first (gives it 'static lifetime via Arc)
        KLIPPER_LIB = Some(Arc::new(lib));

        // Now resolve function symbols from the stored library
        let lib_ref = KLIPPER_LIB.as_ref().unwrap();
        KLIPPER_INIT_FN = Some(
            lib_ref
                .get::<KlipperHostInitFn>(b"klipper_host_init")
                .map_err(|e| {
                    KlippyError::Internal(format!("Failed to resolve klipper_host_init: {}", e))
                })?,
        );
        KLIPPER_INPUT_FN = Some(
            lib_ref
                .get::<KlipperHostInputFn>(b"klipper_host_input")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_input: {}",
                        e
                    ))
                })?,
        );
        KLIPPER_OUTPUT_FN = Some(
            lib_ref
                .get::<KlipperHostOutputFn>(b"klipper_host_output")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_output: {}",
                        e
                    ))
                })?,
        );
        KLIPPER_RUN_FN = Some(
            lib_ref
                .get::<KlipperHostRunFn>(b"klipper_host_run")
                .map_err(|e| {
                    KlippyError::Internal(format!("Failed to resolve klipper_host_run: {}", e))
                })?,
        );
        KLIPPER_STEP_FN = Some(
            lib_ref
                .get::<KlipperHostStepFn>(b"klipper_host_step")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_step: {}",
                        e
                    ))
                })?,
        );
        KLIPPER_SHUTDOWN_FN = Some(
            lib_ref
                .get::<KlipperHostShutdownFn>(b"klipper_host_shutdown")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_shutdown: {}",
                        e
                    ))
                })?,
        );
        KLIPPER_IS_SHUTDOWN_FN = Some(
            lib_ref
                .get::<KlipperHostIsShutdownFn>(b"klipper_host_is_shutdown")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_is_shutdown: {}",
                        e
                    ))
                })?,
        );
        KLIPPER_GET_CLOCK_FN = Some(
            lib_ref
                .get::<KlipperHostGetClockFn>(b"klipper_host_get_clock")
                .map_err(|e| {
                    KlippyError::Internal(format!(
                        "Failed to resolve klipper_host_get_clock: {}",
                        e
                    ))
                })?,
        );
    }

    Ok(())
}

// ============================================================
// Klipper host library function wrappers
// ============================================================

/// Initialize the klipper host library with the given path.
///
/// The path must point to a valid libklipper_host.so file.
/// If the path is empty or invalid, returns an error.
/// Thread-safe and idempotent - calling it multiple times is safe.
pub fn klipper_init(lib_path: &std::path::Path) -> Result<(), KlippyError> {
    // Validate path first
    if !lib_path.exists() {
        return Err(KlippyError::Internal(format!(
            "Klipper host library not found: {}",
            lib_path.display()
        )));
    }

    KLIPPER_INIT.call_once(|| {
        // Load the library first
        if let Err(e) = load_klipper_library(lib_path) {
            warn!("[KLIPPER] Failed to load library: {}", e);
            return;
        }

        unsafe {
            if let Some(init_fn) = &KLIPPER_INIT_FN {
                let ret = init_fn();
                if ret == 0 {
                    KLIPPER_INITIALIZED = true;
                } else {
                    KLIPPER_INITIALIZED = false;
                }
            }
        }
    });

    if unsafe { KLIPPER_INITIALIZED } {
        info!("[KLIPPER] Klipper host library initialized");
        Ok(())
    } else {
        Err(KlippyError::Internal(
            "klipper_host_init() returned failure".into(),
        ))
    }
}

/// Check if the klipper library has been loaded
pub fn klipper_is_loaded() -> bool {
    unsafe { !KLIPPER_LIB.is_none() && KLIPPER_INITIALIZED }
}

/// Run a single iteration of the klipper task loop (non-blocking).
/// Processes pending timers and runs tasks once.
pub fn klipper_run_once() {
    if klipper_is_loaded() {
        unsafe {
            if let Some(step_fn) = &KLIPPER_STEP_FN {
                step_fn();
            }
        }
    }
}

/// Get the current klipper time in clock ticks.
pub fn klipper_time() -> u32 {
    if klipper_is_loaded() {
        unsafe {
            if let Some(clock_fn) = &KLIPPER_GET_CLOCK_FN {
                clock_fn()
            } else {
                0
            }
        }
    } else {
        0
    }
}

/// Check if klipper is in shutdown state.
pub fn klipper_is_shutdown() -> bool {
    if klipper_is_loaded() {
        unsafe {
            if let Some(is_shutdown_fn) = &KLIPPER_IS_SHUTDOWN_FN {
                is_shutdown_fn() != 0
            } else {
                true
            }
        }
    } else {
        true
    }
}

/// Request klipper to shut down.
/// Thread-safe: wakes up klipper_host_run() so it can return.
pub fn klipper_shutdown() {
    if klipper_is_loaded() {
        unsafe {
            if let Some(shutdown_fn) = &KLIPPER_SHUTDOWN_FN {
                shutdown_fn();
            }
        }
    }
}

/// Feed input data into the klipper console.
///
/// Thread-safe: can be called from any thread at any time.
/// The data is copied into an internal buffer and processed
/// by the klipper task loop.
///
/// Returns the number of bytes consumed (always len on success).
pub fn klipper_input(data: &[u8]) -> usize {
    if klipper_is_loaded() {
        unsafe {
            if let Some(input_fn) = &KLIPPER_INPUT_FN {
                input_fn(data.as_ptr(), data.len()) as usize
            } else {
                0
            }
        }
    } else {
        0
    }
}

/// Retrieve output data from the klipper console.
///
/// Thread-safe: can be called from any thread at any time.
/// Returns available output bytes encoded in klipper message format.
///
/// Returns the number of bytes written to buf, 0 if no data available.
pub fn klipper_output(buf: &mut [u8]) -> usize {
    if klipper_is_loaded() {
        unsafe {
            if let Some(output_fn) = &KLIPPER_OUTPUT_FN {
                output_fn(buf.as_mut_ptr(), buf.len())
            } else {
                0
            }
        }
    } else {
        0
    }
}

// ============================================================
// LibInterface
// ============================================================

/// LibInterface provides a real implementation of KlippyInterface
/// using the klipper host library.
///
/// This interface links against the klipper host library dynamically
/// and provides access to Klipper's core functionality through
/// FFI bindings.
pub struct LibInterface {
    handlers: Arc<Mutex<HashMap<InterfaceEvent, Vec<EventHandler>>>>,
    initialized: Arc<Mutex<bool>>,
    lib_path: Mutex<PathBuf>,
    output_buffer: Arc<Mutex<Vec<u8>>>,
    next_seq: AtomicU8,
}

impl LibInterface {
    /// Create a new LibInterface
    pub fn new() -> Self {
        Self {
            handlers: Arc::new(Mutex::new(HashMap::new())),
            initialized: Arc::new(Mutex::new(false)),
            lib_path: Mutex::new(PathBuf::new()),
            output_buffer: Arc::new(Mutex::new(Vec::new())),
            next_seq: AtomicU8::new(0),
        }
    }

    /// Initialize the klipper interface with the given library path.
    ///
    /// The path must point to a valid libklipper_host.so file.
    /// If the path is empty or invalid, returns an error.
    /// The path is stored internally and used for subsequent calls.
    pub fn init(&self, lib_path: &std::path::Path) -> Result<(), KlippyError> {
        // Store the path for future use
        {
            let mut path_guard = self.lib_path.lock().unwrap();
            *path_guard = lib_path.to_path_buf();
        }

        let is_init = self.initialized.lock().unwrap();
        if *is_init {
            return Ok(());
        }
        drop(is_init);

        klipper_init(lib_path)?;

        let mut is_init = self.initialized.lock().unwrap();
        *is_init = true;
        Ok(())
    }

    /// Check if the interface is initialized
    pub fn is_initialized(&self) -> bool {
        *self.initialized.lock().unwrap()
    }

    /// Run one iteration of the klipper task loop (non-blocking).
    pub fn run_once(&self) -> Result<(), KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        klipper_run_once();
        Ok(())
    }

    /// Get the current klipper time in clock ticks.
    pub fn time(&self) -> Result<u32, KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        Ok(klipper_time())
    }

    /// Check if klipper is in shutdown state.
    pub fn is_shutdown(&self) -> Result<bool, KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        Ok(klipper_is_shutdown())
    }

    /// Feed input data into the klipper console.
    ///
    /// The data should be encoded in the klipper binary protocol format.
    pub fn input(&self, data: &[u8]) -> Result<usize, KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        Ok(klipper_input(data))
    }

    /// Retrieve output data from the klipper console.
    ///
    /// Returns available output bytes encoded in klipper message format.
    pub fn output(&self, buf: &mut [u8]) -> Result<usize, KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        let len = klipper_output(buf);
        // Also store in our internal buffer for receive()
        if len > 0 {
            let mut buf_guard = self.output_buffer.lock().unwrap();
            buf_guard.extend_from_slice(&buf[..len]);
        }
        Ok(len)
    }

    /// Request klipper to shut down.
    pub fn shutdown(&self) -> Result<(), KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        klipper_shutdown();
        Ok(())
    }

    /// Internal check: if already initialized, return Ok.
    /// Otherwise, try to initialize with the stored path.
    fn _init_check(&self, lib_path: &PathBuf) -> Result<(), KlippyError> {
        let is_init = self.initialized.lock().unwrap();
        if *is_init {
            return Ok(());
        }
        drop(is_init);
        klipper_init(lib_path)
    }
}

impl Default for LibInterface {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait::async_trait]
impl KlippyInterface for LibInterface {
    async fn send(&self, payload: &Payload) -> Result<(), KlippyError> {
        let path = self.lib_path.lock().unwrap().clone();
        self._init_check(&path)?;
        let data = payload.payload();
        klipper_input(data);
        Ok(())
    }

    async fn receive(&self) -> Result<Payload, crate::core::klippy::traits::InterfaceError> {
        let mut buf = [0u8; 4096];
        let len = self.output(&mut buf).unwrap_or(0);
        if len == 0 {
            return Err(crate::core::klippy::traits::InterfaceError::Timeout);
        }
        let mut p = Payload::new();
        p.push_bytes(&buf[..len]).unwrap();
        Ok(p)
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::KlippyInterface;
    use tracing::debug;

    /// Get the path to the test-built klipper host library.
    /// Provided by the klipperx-test-support dev-dependency,
    /// whose build.rs builds the library during `cargo test`.
    fn test_lib_path() -> std::path::PathBuf {
        klipperx_test_support::klipper_host_lib_path()
    }

    #[test]
    fn test_lib_interface_init() {
        let interface = LibInterface::new();
        assert!(!interface.is_initialized());
        // Use the test-support library path
        interface.init(&test_lib_path()).unwrap();
        assert!(interface.is_initialized());
    }

    #[test]
    fn test_init_with_invalid_path() {
        let interface = LibInterface::new();
        let result =
            interface.init(std::path::Path::new("/nonexistent/path/libklipper_host.so"));
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_frame() {
        let interface = LibInterface::new();
        interface.init(&test_lib_path()).unwrap();
        let payload = Payload::new();
        let result = interface.send(&payload).await;
        assert!(result.is_ok());
    }

    #[test]
    fn test_next_seq() {
        let interface = LibInterface::new();
        assert_eq!(interface.next_seq.load(Ordering::SeqCst), 0);
    }

    // FFI-specific tests
    #[test]
    fn test_klipper_init() {
        klipper_init(&test_lib_path()).unwrap();
    }

    #[test]
    fn test_basic() {
        klipper_init(&test_lib_path()).unwrap();
        let t = klipper_time();
        assert!(t > 0);

        // Test input/output buffers
        let mut buf = [0u8; 256];
        let out_len = klipper_output(&mut buf);
        // May return 0 if no data available - that's fine
        debug!("klipper_output returned {} bytes", out_len);
    }
}
