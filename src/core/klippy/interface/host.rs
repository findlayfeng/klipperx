// Dynamic library interface for KlippyInterface
//
// This module provides a real implementation of KlippyInterface using
// the klipper host library (libklipper_host.so). The library is loaded
// dynamically at runtime via libloading (dlopen/dlsym).
//
// Architecture: Multi-threaded Proxy Pattern
// ===========================================
//
// The Klipper host library's main scheduling loop (klipper_host_run) is
// blocking, so we use a dedicated proxy architecture:
//
//   ┌─────────────────────────────────────────────────────────┐
//   │                     LibInterface                        │
//   │                                                         │
//   │   send() ──┐                                            │
//   │   receive()│                                            │
//   │            │                                            │
//   │   ┌────────▼────────┐                                   │
//   │   │  KlipperProxy   │                                   │
//   │   │                 │                                   │
//   │   │  Channels:      │                                   │
//   │   │  tx_klipper ────┼───► Proxy Thread ◄───┐            │
//   │   │  rx_output ◄────┼───┘                   │            │
//   │   │                 │                       │            │
//   │   │  Ring Buffers:  │                       │            │
//   │   │  rust_to_klippr │◄────► Klipper Thread  │            │
//   │   │  klipper_to_rst │◄────► (runs in C)     │            │
//   └─────────────────────────────────────────────────────────┘
//
// Proxy Thread:
//   - Continuously polls the output ring buffer (klipper_output)
//   - Reads incoming data from tx_klipper channel
//   - Writes data to the rust_to_klipper ring buffer
//   - Reads data from the klipper_to_rust ring buffer
//   - Forwards output data to rx_output channel
//
// Klipper Thread:
//   - Runs klipper_host_run() in a dedicated thread
//   - Uses ring buffers for communication with the proxy thread
//
// This architecture ensures:
//   1. The blocking klipper_host_run() never blocks the application thread
//   2. Multiple producer threads can send data via the channel
//   3. Output data is dispatched to consumers via channel
//   4. No data races on the dynamic library handle

#![allow(static_mut_refs)]

use crate::core::klippy::error::KlippyError;
use crate::core::klippy::traits::{InterfaceEvent, KlippyInterface};
use tracing::{debug, info, warn};

use libloading::Library;
use std::collections::HashMap;
use std::os::raw::{c_uchar, c_long};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

use crate::core::klippy::Payload;
use crossbeam_channel::{Receiver, Sender};

/// Handler callback type
type EventHandler = Arc<dyn Fn(&[u8]) + Send + Sync>;

// ============================================================
// Ring Buffer Constants
// ============================================================

/// Ring buffer size (must be a power of 2 for bitwise masking)
const RING_BUF_SIZE: usize = 1024;

/// Number of bytes to read per proxy loop iteration
const OUTPUT_READ_BATCH: usize = 64;

/// Proxy loop sleep duration (balances CPU usage vs. latency)
const PROXY_LOOP_SLEEP: std::time::Duration = std::time::Duration::from_micros(100);

// ============================================================
// Lock-Free Ring Buffer
// ============================================================

/// A lock-free single-producer single-consumer ring buffer.
///
/// Used for communication between the Rust proxy thread and the
/// Klipper (C) thread. Both head and tail are atomic counters
/// that only grow (never wrap), so the number of items in the
/// buffer is simply `tail - head`.
#[repr(C)]
pub struct RingBuffer {
    /// Circular buffer storage
    buffer: [u8; RING_BUF_SIZE],
    /// Write position (produced)
    tail: AtomicU32,
    /// Read position (consumed)
    head: AtomicU32,
}

impl RingBuffer {
    pub fn new() -> Self {
        Self {
            buffer: [0u8; RING_BUF_SIZE],
            tail: AtomicU32::new(0),
            head: AtomicU32::new(0),
        }
    }

    /// Write bytes into the ring buffer (producer side).
    ///
    /// Returns the number of bytes actually written.
    /// Thread-safe: single producer, uses Release ordering.
    pub fn push_slice(&self, data: &[u8]) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let mut tail = self.tail.load(Ordering::Relaxed);
        let mut pushed = 0;

        for &byte in data {
            if ((tail - head) as usize) < RING_BUF_SIZE {
                let mask = RING_BUF_SIZE - 1;
                let ptr = self.buffer.as_ptr() as *mut u8;
                unsafe {
                    ptr.add(tail as usize & mask).write(byte);
                }
                tail += 1;
                pushed += 1;
            } else {
                break; // Buffer full
            }
        }

        self.tail.store(tail, Ordering::Release);
        pushed
    }

    /// Read bytes from the ring buffer (consumer side).
    ///
    /// Returns up to `max_bytes` bytes.
    /// Thread-safe: single consumer, uses Acquire ordering.
    pub fn pop_to_vec(&self, max_bytes: usize) -> Vec<u8> {
        let tail = self.tail.load(Ordering::Acquire);
        let mut head = self.head.load(Ordering::Relaxed);
        let mut result = Vec::with_capacity(max_bytes.min((tail - head) as usize));

        let mask = RING_BUF_SIZE - 1;
        while head != tail && result.len() < max_bytes {
            let byte = self.buffer[head as usize & mask];
            result.push(byte);
            head += 1;
        }

        self.head.store(head, Ordering::Relaxed);
        result
    }
}

// ============================================================
// Klipper Host Library FFI
// ============================================================

/// Klipper host library function pointer types
type KlipperHostInitFn = unsafe extern "C" fn() -> c_long;
type KlipperHostInputFn = unsafe extern "C" fn(*const c_uchar, usize) -> c_long;
type KlipperHostOutputFn = unsafe extern "C" fn(*mut c_uchar, usize) -> usize;
type KlipperHostRunFn = unsafe extern "C" fn();
type KlipperHostShutdownFn = unsafe extern "C" fn();
type KlipperHostIsShutdownFn = unsafe extern "C" fn() -> c_uchar;
type KlipperHostGetClockFn = unsafe extern "C" fn() -> u32;

/// The realtime loop function provided by the C library.
/// Receives ring buffer pointers for bidirectional communication.
type CRealtimeLoopFunc = unsafe extern "C" fn(*const RingBuffer, *const RingBuffer);

// ============================================================
// Global Library State (Legacy klipper_* API)
// ============================================================

/// Global library handle and function pointers for the legacy klipper_* API.
/// Used by klipper_init(), klipper_input(), etc. for backward compatibility.
static mut KLIPPER_LIB: Option<Arc<Library>> = None;
static mut KLIPPER_INIT_FN: Option<unsafe extern "C" fn() -> c_long> = None;
static mut KLIPPER_INPUT_FN: Option<unsafe extern "C" fn(*const c_uchar, usize) -> c_long> = None;
static mut KLIPPER_OUTPUT_FN: Option<unsafe extern "C" fn(*mut c_uchar, usize) -> usize> = None;
static mut KLIPPER_RUN_FN: Option<unsafe extern "C" fn()> = None;
static mut KLIPPER_SHUTDOWN_FN: Option<unsafe extern "C" fn()> = None;
static mut KLIPPER_IS_SHUTDOWN_FN: Option<unsafe extern "C" fn() -> c_uchar> = None;
static mut KLIPPER_GET_CLOCK_FN: Option<unsafe extern "C" fn() -> u32> = None;
static mut KLIPPER_INITIALIZED: bool = false;
static KLIPPER_INIT_ONCE: std::sync::Once = std::sync::Once::new();

// ============================================================
// Dynamic library loader (internal)
// ============================================================

/// Internal: load the klipper host shared library.
///
/// Returns the Arc-wrapped Library. Call `resolve_symbols` separately
/// to get function pointers (which require 'static lifetime from a
/// library that lives for the entire program).
fn load_library(lib_path: &std::path::Path) -> Result<Arc<Library>, KlippyError> {
    let lib = Arc::new(unsafe {
        Library::new(lib_path).map_err(|e| {
            KlippyError::Internal(format!(
                "Failed to load libklipper_host.so: {} (path: {})",
                e,
                lib_path.display()
            ))
        })?
    });

    info!("[KLIPPER] Library loaded from: {}", lib_path.display());
    Ok(lib)
}

/// Resolve all function symbols from a library.
/// Returns raw function pointers (no lifetime).
fn resolve_symbols(lib: &Arc<Library>) -> Result<ResolveHandles, KlippyError> {
    let lib_ref: &Library = lib;
    Ok(ResolveHandles {
        init: unsafe { *(*lib_ref).get::<KlipperHostInitFn>(b"klipper_host_init")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_init: {}", e)))? },
        input: unsafe { *(*lib_ref).get::<KlipperHostInputFn>(b"klipper_host_input")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_input: {}", e)))? },
        output: unsafe { *(*lib_ref).get::<KlipperHostOutputFn>(b"klipper_host_output")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_output: {}", e)))? },
        run: unsafe { *(*lib_ref).get::<KlipperHostRunFn>(b"klipper_host_run")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_run: {}", e)))? },
        shutdown: unsafe { *(*lib_ref).get::<KlipperHostShutdownFn>(b"klipper_host_shutdown")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_shutdown: {}", e)))? },
        is_shutdown: unsafe { *(*lib_ref).get::<KlipperHostIsShutdownFn>(b"klipper_host_is_shutdown")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_is_shutdown: {}", e)))? },
        get_clock: unsafe { *(*lib_ref).get::<KlipperHostGetClockFn>(b"klipper_host_get_clock")
            .map_err(|e| KlippyError::Internal(format!("Failed to resolve klipper_host_get_clock: {}", e)))? },
        rt_loop: unsafe { (*lib_ref).get::<CRealtimeLoopFunc>(b"run_realtime_loop\0")
            .map(|s| *s).ok() },
    })
}

/// Resolved function pointers for a library instance.
/// Uses raw function pointers — the library must outlive usage.
#[derive(Clone)]
struct ResolveHandles {
    init: unsafe extern "C" fn() -> c_long,
    input: unsafe extern "C" fn(*const c_uchar, usize) -> c_long,
    output: unsafe extern "C" fn(*mut c_uchar, usize) -> usize,
    run: unsafe extern "C" fn(),
    shutdown: unsafe extern "C" fn(),
    is_shutdown: unsafe extern "C" fn() -> c_uchar,
    get_clock: unsafe extern "C" fn() -> u32,
    rt_loop: Option<unsafe extern "C" fn(*const RingBuffer, *const RingBuffer)>,
}

// ============================================================
// Legacy klipper_* API (backward compatible)
// ============================================================

pub fn klipper_init(lib_path: &std::path::Path) -> Result<(), KlippyError> {
    if !lib_path.exists() {
        return Err(KlippyError::Internal(format!(
            "Klipper host library not found: {}",
            lib_path.display()
        )));
    }

    KLIPPER_INIT_ONCE.call_once(|| {
        unsafe {
            if KLIPPER_LIB.is_some() {
                return;
            }
        }

        match load_library(lib_path) {
            Ok(lib) => {
                unsafe {
                    KLIPPER_LIB = Some(lib);
                    if let Ok(handles) = resolve_symbols(&KLIPPER_LIB.as_ref().unwrap()) {
                        KLIPPER_INIT_FN = Some(handles.init);
                        KLIPPER_INPUT_FN = Some(handles.input);
                        KLIPPER_OUTPUT_FN = Some(handles.output);
                        KLIPPER_RUN_FN = Some(handles.run);
                        KLIPPER_SHUTDOWN_FN = Some(handles.shutdown);
                        KLIPPER_IS_SHUTDOWN_FN = Some(handles.is_shutdown);
                        KLIPPER_GET_CLOCK_FN = Some(handles.get_clock);
                        let ret = KLIPPER_INIT_FN.unwrap();
                        KLIPPER_INITIALIZED = (ret)() == 0;
                    }
                }
            }
            Err(e) => {
                warn!("[KLIPPER] Failed to load library: {}", e);
            }
        }
    });

    unsafe {
        if KLIPPER_INITIALIZED {
            info!("[KLIPPER] Klipper host library initialized");
            Ok(())
        } else {
            Err(KlippyError::Internal("klipper_host_init() returned failure".into()))
        }
    }
}

pub fn klipper_is_loaded() -> bool {
    unsafe { !KLIPPER_LIB.is_none() && KLIPPER_INITIALIZED }
}

pub fn klipper_run_once() {
    if klipper_is_loaded() {
        unsafe {
            if let Some(fn_) = KLIPPER_RUN_FN {
                fn_();
            }
        }
    }
}

pub fn klipper_time() -> u32 {
    if klipper_is_loaded() {
        unsafe {
            if let Some(fn_) = KLIPPER_GET_CLOCK_FN {
                fn_()
            } else {
                0
            }
        }
    } else {
        0
    }
}

pub fn klipper_is_shutdown() -> bool {
    if klipper_is_loaded() {
        unsafe {
            if let Some(fn_) = KLIPPER_IS_SHUTDOWN_FN {
                fn_() != 0
            } else {
                true
            }
        }
    } else {
        true
    }
}

pub fn klipper_shutdown() {
    if klipper_is_loaded() {
        unsafe {
            if let Some(fn_) = KLIPPER_SHUTDOWN_FN {
                fn_();
            }
        }
    }
}

pub fn klipper_input(data: &[u8]) -> usize {
    if klipper_is_loaded() {
        debug!(
            "[KLIPPER] INPUT {} bytes: {}",
            data.len(),
            hex::encode(&data[..data.len().min(200)])
        );
        unsafe {
            if let Some(fn_) = KLIPPER_INPUT_FN {
                fn_(data.as_ptr(), data.len()) as usize
            } else {
                0
            }
        }
    } else {
        0
    }
}

pub fn klipper_output(buf: &mut [u8]) -> usize {
    if klipper_is_loaded() {
        unsafe {
            if let Some(fn_) = KLIPPER_OUTPUT_FN {
                let len = fn_(buf.as_mut_ptr(), buf.len());
                if len > 0 {
                    debug!(
                        "[KLIPPER] OUTPUT {} bytes: {}",
                        len,
                        hex::encode(&buf[..len.min(200)])
                    );
                }
                len
            } else {
                0
            }
        }
    } else {
        0
    }
}

// ============================================================
// Klipper Proxy — Multi-threaded Bridge
// ============================================================

/// KlipperProxy manages the proxy thread and Klipper thread.
pub struct KlipperProxy {
    tx_klipper: Sender<Vec<u8>>,
    rx_output: Receiver<Vec<u8>>,
    rust_to_klipper: Arc<RingBuffer>,
    klipper_to_rust: Arc<RingBuffer>,
    /// Library handle — kept alive for the lifetime of the proxy
    _lib: Arc<Library>,
    handles: ResolveHandles,
    proxy_handle: Mutex<Option<std::thread::JoinHandle<()>>>,
    klipper_handle: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl KlipperProxy {
    pub fn new(lib_path: &std::path::Path) -> Result<Self, KlippyError> {
        let lib = load_library(lib_path)?;

        // Resolve symbols — dereferencing Symbol gives raw fn pointer
        let handles = resolve_symbols(&lib)?;

        // Initialize the klipper host library
        unsafe {
            let ret = (handles.init)();
            if ret != 0 {
                return Err(KlippyError::Internal("klipper_host_init() returned failure".into()));
            }
        }

        let rust_to_klipper = Arc::new(RingBuffer::new());
        let klipper_to_rust = Arc::new(RingBuffer::new());

        let (tx_klipper, rx_from_rust) = crossbeam_channel::unbounded::<Vec<u8>>();
        let (tx_output, rx_output) = crossbeam_channel::unbounded::<Vec<u8>>();

        // Capture rt_loop for the Klipper thread (before handles is moved)
        let klipper_rt_loop = handles.rt_loop;
        // Proxy uses its own handles (not the legacy globals)
        let proxy_handles = Arc::new(handles.clone());
        let proxy_rtk = Arc::clone(&rust_to_klipper);
        let proxy_ktr = Arc::clone(&klipper_to_rust);

        // Spawn the proxy thread
        let proxy_handle = std::thread::Builder::new()
            .name("klipper-proxy".to_string())
            .spawn(move || {
                info!("[KLIPPER] Proxy thread started");
                let mut out_buf = [0u8; 2048];

                loop {
                    // A. Forward data from Rust threads → Klipper ring buffer
                    crossbeam_channel::select! {
                        recv(rx_from_rust) -> msg => {
                            if let Ok(data) = msg {
                                if !data.is_empty() {
                                    let written = proxy_rtk.push_slice(&data);
                                    if written > 0 {
                                        debug!("[KLIPPER Proxy] Forwarded {} bytes to Klipper", written);
                                    }
                                }
                            }
                        }
                        default => {}
                    }

                    // B. Poll Klipper output (using proxy's own handles) and push to output ring buffer
                    let bytes_read = unsafe { (proxy_handles.output)(out_buf.as_mut_ptr(), out_buf.len()) };
                    if bytes_read > 0 {
                        let _ = proxy_ktr.push_slice(&out_buf[..bytes_read]);
                    }

                    // C. Read from output ring buffer and send to consumers
                    let output_data = proxy_ktr.pop_to_vec(OUTPUT_READ_BATCH);
                    if !output_data.is_empty() {
                        let _ = tx_output.send(output_data);
                    }

                    std::thread::sleep(PROXY_LOOP_SLEEP);
                }
            })
            .map_err(|e| KlippyError::Internal(format!("Failed to spawn proxy thread: {}", e)))?;

        let klipper_rtk = Arc::clone(&rust_to_klipper);
        let klipper_ktr = Arc::clone(&klipper_to_rust);
        // Check if we own the Arc (can convert to raw pointer without freeing)
        let rtk_can_leak = Arc::strong_count(&klipper_rtk) == 1;
        let ktr_can_leak = Arc::strong_count(&klipper_ktr) == 1;

        // Spawn the Klipper thread
        let klipper_handle = std::thread::Builder::new()
            .name("klipper-runtime".to_string())
            .spawn(move || {
                info!("[KLIPPER] Klipper thread started");
                // Convert Arc to raw pointer for FFI
                let rtk_ptr = if rtk_can_leak {
                    Arc::into_inner(klipper_rtk)
                        .map(|rb| Box::into_raw(Box::new(rb)))
                        .unwrap_or_else(|| {
                            let rb = RingBuffer::new();
                            Box::into_raw(Box::new(rb))
                        })
                } else {
                    let rb = RingBuffer::new();
                    Box::into_raw(Box::new(rb))
                };
                let ktr_ptr = if ktr_can_leak {
                    Arc::into_inner(klipper_ktr)
                        .map(|rb| Box::into_raw(Box::new(rb)))
                        .unwrap_or_else(|| {
                            let rb = RingBuffer::new();
                            Box::into_raw(Box::new(rb))
                        })
                } else {
                    let rb = RingBuffer::new();
                    Box::into_raw(Box::new(rb))
                };
                // Run the realtime loop using the captured rt_loop function
                if let Some(rt_loop_fn) = klipper_rt_loop {
                    unsafe { rt_loop_fn(rtk_ptr, ktr_ptr); }
                    info!("[KLIPPER] Realtime loop exited (ring buffer mode)");
                } else {
                    // Fallback: run klipper_host_run() in a loop
                    loop {
                        klipper_run_once();
                        if klipper_is_shutdown() { break; }
                        std::thread::sleep(std::time::Duration::from_micros(100));
                    }
                    info!("[KLIPPER] klipper_host_run() exited (fallback mode)");
                }
                // Clean up the ring buffer
                unsafe { drop(Box::from_raw(rtk_ptr)); }
                unsafe { drop(Box::from_raw(ktr_ptr)); }
            })
            .map_err(|e| KlippyError::Internal(format!("Failed to spawn Klipper thread: {}", e)))?;

        Ok(KlipperProxy {
            tx_klipper,
            rx_output,
            rust_to_klipper,
            klipper_to_rust,
            _lib: lib,
            handles: handles.clone(),
            proxy_handle: Mutex::new(Some(proxy_handle)),
            klipper_handle: Mutex::new(Some(klipper_handle)),
        })
    }

    /// Send data to the Klipper thread via the proxy.
    pub fn send(&self, data: &[u8]) -> Result<(), KlippyError> {
        self.tx_klipper
            .send(data.to_vec())
            .map_err(|_| KlippyError::Internal("Klipper proxy channel closed".into()))?;
        Ok(())
    }

    /// Receive data from the Klipper thread with a timeout.
    pub fn receive(&self, timeout: std::time::Duration) -> Result<Vec<u8>, crate::core::klippy::traits::InterfaceError> {
        let rx = self.rx_output.clone();
        match std::thread::spawn(move || rx.recv_timeout(timeout))
            .join()
            .map_err(|_| {
                crate::core::klippy::traits::InterfaceError::Other("receive thread panicked".into())
            })? {
            Ok(data) => Ok(data),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                Err(crate::core::klippy::traits::InterfaceError::Timeout)
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                Err(crate::core::klippy::traits::InterfaceError::ConnectionLost)
            }
        }
    }

    /// Receive data with the default timeout (1 second).
    pub fn receive_default(&self) -> Result<Vec<u8>, crate::core::klippy::traits::InterfaceError> {
        self.receive(std::time::Duration::from_secs(1))
    }

    pub fn is_shutdown(&self) -> bool {
        klipper_is_shutdown()
    }

    pub fn shutdown(&self) {
        klipper_shutdown();
    }

    pub fn get_clock(&self) -> u32 {
        klipper_time()
    }
}

impl Drop for KlipperProxy {
    fn drop(&mut self) {
        self.shutdown();
        if let Some(handle) = self.klipper_handle.lock().unwrap().take() {
            let _ = handle.join();
        }
        if let Some(handle) = self.proxy_handle.lock().unwrap().take() {
            let _ = handle.join();
        }
        info!("[KLIPPER] Proxy and Klipper threads joined");
        // _lib is dropped automatically, freeing the library
    }
}

// ============================================================
// KlippyInterface Implementation
// ============================================================

/// LibInterface provides a real implementation of KlippyInterface
/// using the klipper host library with a multi-threaded proxy architecture.
pub struct LibInterface {
    proxy: Arc<Mutex<Option<KlipperProxy>>>,
    handlers: Arc<Mutex<HashMap<InterfaceEvent, Vec<EventHandler>>>>,
    initialized: Arc<Mutex<bool>>,
    lib_path: Mutex<PathBuf>,
    output_buffer: Arc<Mutex<Vec<u8>>>,
    next_seq: AtomicU8,
}

impl LibInterface {
    pub fn new() -> Self {
        Self {
            proxy: Arc::new(Mutex::new(None)),
            handlers: Arc::new(Mutex::new(HashMap::new())),
            initialized: Arc::new(Mutex::new(false)),
            lib_path: Mutex::new(PathBuf::new()),
            output_buffer: Arc::new(Mutex::new(Vec::new())),
            next_seq: AtomicU8::new(0),
        }
    }

    pub fn init(&self, lib_path: &std::path::Path) -> Result<(), KlippyError> {
        {
            let mut path_guard = self.lib_path.lock().unwrap();
            *path_guard = lib_path.to_path_buf();
        }

        {
            let is_init = self.initialized.lock().unwrap();
            if *is_init {
                return Ok(());
            }
        }

        let proxy = KlipperProxy::new(lib_path)?;
        {
            let mut proxy_guard = self.proxy.lock().unwrap();
            *proxy_guard = Some(proxy);
        }

        {
            let mut is_init = self.initialized.lock().unwrap();
            *is_init = true;
        }

        info!("[KLIPPER] Klipper interface initialized");
        Ok(())
    }

    pub fn is_initialized(&self) -> bool {
        *self.initialized.lock().unwrap()
    }

    pub fn run_once(&self) -> Result<(), KlippyError> {
        self.ensure_proxy()?;
        Ok(())
    }

    pub fn time(&self) -> Result<u32, KlippyError> {
        self.ensure_proxy()?;
        let proxy = self.proxy.lock().unwrap();
        Ok(proxy.as_ref().map(|p| p.get_clock()).unwrap_or(0))
    }

    pub fn is_shutdown(&self) -> Result<bool, KlippyError> {
        self.ensure_proxy()?;
        let proxy = self.proxy.lock().unwrap();
        Ok(proxy.as_ref().map(|p| p.is_shutdown()).unwrap_or(true))
    }

    pub fn input(&self, data: &[u8]) -> Result<usize, KlippyError> {
        self.ensure_proxy()?;
        let proxy = self.proxy.lock().unwrap();
        proxy.as_ref().map(|p| p.send(data)).unwrap_or(Ok(()))?;
        Ok(data.len())
    }

    pub fn output(&self, buf: &mut [u8]) -> Result<usize, KlippyError> {
        self.ensure_proxy()?;
        let proxy = self.proxy.lock().unwrap();
        let proxy_ref = proxy.as_ref().ok_or_else(|| {
            KlippyError::Internal("Proxy not initialized".into())
        })?;

        match proxy_ref.receive_default() {
            Ok(data) => {
                let copy_len = data.len().min(buf.len());
                buf[..copy_len].copy_from_slice(&data[..copy_len]);
                if copy_len > 0 {
                    let mut buf_guard = self.output_buffer.lock().unwrap();
                    buf_guard.extend_from_slice(&data[..copy_len]);
                }
                debug!("[KLIPPER] OUTPUT {} bytes: {}", copy_len, hex::encode(&buf[..copy_len.min(200)]));
                Ok(copy_len)
            }
            Err(_) => Ok(0),
        }
    }

    pub fn shutdown(&self) -> Result<(), KlippyError> {
        {
            let mut proxy_guard = self.proxy.lock().unwrap();
            if let Some(proxy) = proxy_guard.as_mut() {
                proxy.shutdown();
            }
        }
        Ok(())
    }

    fn ensure_proxy(&self) -> Result<(), KlippyError> {
        {
            let proxy = self.proxy.lock().unwrap();
            if proxy.is_some() {
                return Ok(());
            }
        }

        let lib_path = self.lib_path.lock().unwrap().clone();
        if lib_path.as_os_str().is_empty() {
            return Err(KlippyError::Internal("Library path not set. Call init() first.".into()));
        }

        let proxy = KlipperProxy::new(&lib_path)?;
        {
            let mut proxy_guard = self.proxy.lock().unwrap();
            *proxy_guard = Some(proxy);
        }

        let mut is_init = self.initialized.lock().unwrap();
        *is_init = true;

        Ok(())
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
        let data = payload.payload();
        self.input(data).map(|_| ())
    }

    async fn receive(&self) -> Result<Payload, crate::core::klippy::traits::InterfaceError> {
        if let Err(e) = self.ensure_proxy() {
            return Err(crate::core::klippy::traits::InterfaceError::Other(e.to_string()));
        }

        // Clone the proxy handle for the async context
        let proxy = self.proxy.clone();

        let result = tokio::task::spawn_blocking(move || {
            let guard = proxy.lock().unwrap();
            let proxy_ref = guard.as_ref().ok_or_else(|| {
                crate::core::klippy::traits::InterfaceError::ConnectionLost
            })?;
            proxy_ref.receive(std::time::Duration::from_secs(1))
        })
        .await
        .map_err(|_| {
            crate::core::klippy::traits::InterfaceError::Other("receive task panicked".into())
        })?;

        match result {
            Ok(bytes) => {
                let mut p = Payload::new();
                p.push_bytes(&bytes).map_err(|_| {
                    crate::core::klippy::traits::InterfaceError::InvalidData
                })?;
                Ok(p)
            }
            Err(e) => Err(e),
        }
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

    fn test_lib_path() -> std::path::PathBuf {
        klipperx_test_support::klipper_host_lib_path()
    }

    #[test]
    #[ignore]
    fn test_lib_interface_init() {
        let interface = LibInterface::new();
        assert!(!interface.is_initialized());
        interface.init(&test_lib_path()).unwrap();
        assert!(interface.is_initialized());
    }

    #[test]
    #[ignore]
    fn test_init_with_invalid_path() {
        let interface = LibInterface::new();
        let result = interface.init(std::path::Path::new("/nonexistent/path/libklipper_host.so"));
        assert!(result.is_err());
    }

    #[tokio::test]
    #[ignore]
    async fn test_send_frame() {
        let interface = LibInterface::new();
        interface.init(&test_lib_path()).unwrap();
        let payload = Payload::new();
        let result = interface.send(&payload).await;
        assert!(result.is_ok());
    }

    #[test]
    #[ignore]
    fn test_next_seq() {
        let interface = LibInterface::new();
        assert_eq!(interface.next_seq.load(Ordering::SeqCst), 0);
    }

    #[test]
    #[ignore]
    fn test_klipper_init() {
        klipper_init(&test_lib_path()).unwrap();
    }

    #[test]
    #[ignore]
    fn test_basic() {
        klipper_init(&test_lib_path()).unwrap();
        let t = klipper_time();
        assert!(t > 0);

        let mut buf = [0u8; 256];
        let out_len = klipper_output(&mut buf);
        debug!("klipper_output returned {} bytes", out_len);
    }
}

// ============================================================
// RingBuffer Unit Tests
// ============================================================

#[cfg(test)]
mod ring_buffer_tests {
    use super::*;

    #[test]
    fn test_new_empty() {
        let rb = RingBuffer::new();
        let data = rb.pop_to_vec(10);
        assert!(data.is_empty());
    }

    #[test]
    fn test_push_pop_single_byte() {
        let rb = RingBuffer::new();
        let pushed = rb.push_slice(&[42]);
        assert_eq!(pushed, 1);
        let data = rb.pop_to_vec(10);
        assert_eq!(data, vec![42]);
    }

    #[test]
    fn test_push_pop_multiple_bytes() {
        let rb = RingBuffer::new();
        let input: Vec<u8> = (0..20).collect();
        let pushed = rb.push_slice(&input);
        assert_eq!(pushed, 20);
        let data = rb.pop_to_vec(100);
        assert_eq!(data, input);
    }

    #[test]
    fn test_pop_empty_returns_empty() {
        let rb = RingBuffer::new();
        let data = rb.pop_to_vec(10);
        assert!(data.is_empty());
        // Second pop also returns empty
        let data2 = rb.pop_to_vec(10);
        assert!(data2.is_empty());
    }

    #[test]
    fn test_push_respects_max_bytes() {
        let rb = RingBuffer::new();
        let input: Vec<u8> = (0..100).collect();
        let data = rb.pop_to_vec(10);
        assert_eq!(data.len(), 0); // Nothing pushed yet
        let pushed = rb.push_slice(&input);
        assert_eq!(pushed, 100);
        let data = rb.pop_to_vec(10);
        assert_eq!(data.len(), 10); // Limited to 10
    }

    #[test]
    fn test_full_buffer_stops_pushing() {
        let rb = RingBuffer::new();
        // Fill the buffer
        let input: Vec<u8> = (0..RING_BUF_SIZE).map(|i| i as u8).collect();
        assert_eq!(input.len(), RING_BUF_SIZE);
        let pushed = rb.push_slice(&input);
        assert_eq!(pushed, RING_BUF_SIZE);
        // Buffer is full, next push should write 0 bytes
        let pushed2 = rb.push_slice(&[1, 2, 3]);
        assert_eq!(pushed2, 0);
    }

    #[test]
    fn test_data_integrity() {
        let rb = RingBuffer::new();
        // Push various patterns
        let patterns = vec![
            vec![0u8; 10],
            vec![255u8; 10],
            (0..50).map(|i| i as u8).collect::<Vec<u8>>(),
            vec![1, 2, 3, 4, 5],
        ];
        for pattern in &patterns {
            let pushed = rb.push_slice(pattern);
            assert_eq!(pushed, pattern.len());
        }
        // Pop all data
        let total_len: usize = patterns.iter().map(|p| p.len()).sum();
        let data = rb.pop_to_vec(total_len);
        let expected: Vec<u8> = patterns.concat();
        assert_eq!(data, expected);
    }

    #[test]
    fn test_wrap_around() {
        let rb = RingBuffer::new();
        // Push and pop enough to wrap around
        for i in 0..(RING_BUF_SIZE * 2) {
            let byte = (i % 256) as u8;
            let pushed = rb.push_slice(&[byte]);
            assert_eq!(pushed, 1);
            let data = rb.pop_to_vec(1);
            assert_eq!(data, vec![byte]);
        }
    }

    #[test]
    fn test_multiple_cycles() {
        let rb = RingBuffer::new();
        for cycle in 0..10 {
            let input: Vec<u8> = (0..(cycle + 1) as u8).collect();
            let pushed = rb.push_slice(&input);
            assert_eq!(pushed, input.len());
            let data = rb.pop_to_vec(100);
            assert_eq!(data, input);
        }
    }

    #[test]
    fn test_push_slice_partial() {
        let rb = RingBuffer::new();
        // Fill most of the buffer
        let input1: Vec<u8> = (0..(RING_BUF_SIZE - 5)).map(|i| i as u8).collect();
        assert_eq!(input1.len(), RING_BUF_SIZE - 5);
        let pushed1 = rb.push_slice(&input1);
        assert_eq!(pushed1, RING_BUF_SIZE - 5);
        // Try to push more than remaining space
        let input2: Vec<u8> = (0..100).map(|i| i as u8).collect();
        let pushed2 = rb.push_slice(&input2);
        assert_eq!(pushed2, 5); // Only 5 bytes fit
        // Verify data
        let data = rb.pop_to_vec(RING_BUF_SIZE);
        assert_eq!(data.len(), RING_BUF_SIZE);
        // First part should be input1
        assert_eq!(&data[..input1.len()], &input1);
        // Second part should be first 5 bytes of input2
        assert_eq!(&data[input1.len()..], &input2[..5]);
    }

    #[test]
    fn test_ring_buf_size_is_power_of_two() {
        // RING_BUF_SIZE must be a power of 2 for bitwise masking to work as modulo
        assert!(RING_BUF_SIZE > 0);
        assert_eq!(RING_BUF_SIZE & (RING_BUF_SIZE - 1), 0);
    }
}
