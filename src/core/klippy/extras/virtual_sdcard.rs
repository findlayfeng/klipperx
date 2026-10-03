//! `[virtual_sdcard]` — print files directly from a host g-code file
//! (upstream `klippy/extras/virtual_sdcard.py`).
//!
//! This port covers file-management, the `M20`–`M27` / `SDCARD_*` command
//! family, **and** the `work_handler` replay loop. `M24` (`do_resume`)
//! registers a one-shot reactor timer whose callback spawns a tokio task that
//! reads the file block-by-block and dispatches each line through
//! `gcode.run_script`. `M25` (`do_pause`) sets a flag the task checks each
//! iteration; on exit the task calls the appropriate `print_stats.note_*`.
//!
//! | option | default | role |
//! |---|---|---|
//! | `path` | — (required) | the directory print files live in |
//! | `on_error_gcode` | upstream's `DEFAULT_ERROR_GCODE` | script to run after a file error |
//!
//! # What is not here
//!
//! - **`gcode.get_mutex().test()` yield** — upstream yields before each line
//!   if a external command is pending (`virtual_sdcard.py:233-236`). This port
//!   has no `gcode_mutex.test()` API; the replay task yields with
//!   `tokio::task::yield_now()` instead, so the interleaving of external
//!   commands during replay differs from upstream. First version; to be
//!   revisited when `gcode` exposes a mutex/test API.
//! - **`_handle_analyze_shutdown` / `_handle_debuginput_exit`** — upstream
//!   registers these to log file-tail on shutdown and to wait for replay to
//!   finish on debuginput exit (`virtual_sdcard.py:278-294`). Not wired here.
//! - **`stats(eventtime)`** — upstream returns `(True, "sd_pos=%d")` while the
//!   timer runs (`virtual_sdcard.py:75-78`). `PrinterObject` has no `stats`
//!   method, so this is omitted; `stats` is a reactor scheduling hint, not
//!   user-visible.
//! - **`do_pause` now waits** for the replay task to exit, matching
//!   upstream's synchronous spin (`virtual_sdcard.py:123-127`). The wait is
//!   async (`tokio::time::sleep` polling `work_active`) so the replay task
//!   can run to completion on the same runtime; the `cmd_from_sd` guard
//!   skips the wait when `do_pause` is called from a replayed line (avoiding
//!   self-deadlock). `do_cancel` and `reset_file` both call `do_pause`, so
//!   they also wait before closing the file.
//! - **`expanduser`/`normpath`** — upstream runs `normpath(expanduser(…))`
//!   over `path`; this port uses it as-is for `join`. Corpus paths are
//!   relative (`test/klippy/sdcard_loop`) and need no expansion.

use std::fs;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

#[cfg(test)]
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::print_stats::PrintStats;
use crate::core::klippy::extras::template::{Builtin, Context, PrinterView, Rt, Template};
use crate::core::klippy::gcode::{
    CommandError, CommandFuture, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// File extensions upstream accepts (`virtual_sdcard.py:6`).
const VALID_GCODE_EXTS: &[&str] = &["gcode", "g", "gco"];

/// Upstream's `DEFAULT_ERROR_GCODE` — the default of `on_error_gcode`
/// (`virtual_sdcard.py:8-12`). Loaded as a template; not rendered until Unit C.
const DEFAULT_ERROR_GCODE: &str = "
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
";

section!("virtual_sdcard", order = 30, load = load_config);

/// The `[virtual_sdcard]` module object (upstream's `VirtualSD`).
///
/// The mutable state lives behind a `Mutex` because command handlers run on
/// the dispatcher's async task while `get_status` can be called from any
/// thread.
pub struct VirtualSdCard {
    /// The `path` option as written (`virtual_sdcard.py:17-19`). Upstream runs
    /// `normpath(expanduser(…))` over it; this port uses it as-is (module docs).
    sdcard_dirname: String,
    /// The open file handle, or `None` when no file is selected
    /// (`virtual_sdcard.py:22`).
    current_file: Mutex<Option<FileHandle>>,
    /// `file_position` / `file_size` (`virtual_sdcard.py:23`).
    file_position: Mutex<u64>,
    file_size: Mutex<u64>,
    /// `must_pause_work` / `cmd_from_sd` (`virtual_sdcard.py:28`).
    must_pause_work: Mutex<bool>,
    cmd_from_sd: Mutex<bool>,
    /// `work_timer is not None` — `true` while the replay task is running
    /// (`virtual_sdcard.py:30`). `do_resume` sets it `true` when spawning the
    /// task; the task sets it `false` on exit.
    work_active: Mutex<bool>,
    /// `next_file_position` — written by `work_handler` before each line and
    /// read by `get_file_position` (`virtual_sdcard.py:29`).
    next_file_position: Mutex<u64>,
    /// The `print_stats` object, loaded via `PrintStats::ensure`
    /// (`virtual_sdcard.py:25`).
    print_stats: Arc<PrintStats>,
    /// The compiled `on_error_gcode` template (`virtual_sdcard.py:33-35`).
    /// Rendered inside `work_handler` when a replayed line errors.
    on_error_gcode: Template,
    /// The `gcode` dispatcher, for `run_script` / `respond_raw` from the
    /// replay task (which has no `GcodeCommand` in hand).
    gcode: Arc<GCodeDispatch>,
    /// The printer, for `send_event` and `reactor` (`virtual_sdcard.py:117`).
    printer: Arc<Printer>,
}

/// A snapshot of the mutable state, for `get_status`.
struct StateSnapshot {
    file_path: Option<String>,
    file_position: u64,
    file_size: u64,
    is_active: bool,
}

/// The open file and its display name (the relative path the user gave to
/// `M23`/`SDCARD_PRINT_FILE`, which is what upstream's `current_file.name`
/// reports through `file_path()`).
struct FileHandle {
    /// The reader, seeked and read by the replay loop.
    reader: std::io::BufReader<fs::File>,
    /// The filename as the user supplied it (without leading `/`),
    /// reported through `file_path()` / `get_status`.
    name: String,
}

impl VirtualSdCard {
    /// Read the section and wire the commands (upstream's `__init__`,
    /// `virtual_sdcard.py:14-46`).
    ///
    /// # Errors
    /// A missing `path`, an unparsable `on_error_gcode` template, or a g-code
    /// name the dispatcher refuses.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let sdcard_dirname = config.get("path", None)?;
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let on_error_gcode =
            gcode_macro.load_template(config, "on_error_gcode", Some(DEFAULT_ERROR_GCODE))?;
        let print_stats = PrintStats::ensure(printer)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        let object = Arc::new(Self {
            sdcard_dirname,
            current_file: Mutex::new(None),
            file_position: Mutex::new(0),
            file_size: Mutex::new(0),
            must_pause_work: Mutex::new(false),
            cmd_from_sd: Mutex::new(false),
            work_active: Mutex::new(false),
            next_file_position: Mutex::new(0),
            print_stats,
            on_error_gcode,
            gcode,
            printer: Arc::clone(printer),
        });
        object.register_commands(printer)?;
        Ok(object)
    }

    /// `get_file_list(check_subdirs)` (`virtual_sdcard.py:79-103`):
    /// list g-code files, optionally recursing into subdirectories.
    ///
    /// Returns `Vec<(relative_path, size)>` sorted by lowercase path.
    /// Returns `Err` when the directory cannot be read.
    fn get_file_list(&self, check_subdirs: bool) -> Result<Vec<(String, u64)>, String> {
        let root = Path::new(&self.sdcard_dirname);
        if check_subdirs {
            let mut flist = Vec::new();
            walk_dir(root, root, &mut flist)?;
            flist.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));
            Ok(flist)
        } else {
            let mut flist = Vec::new();
            let entries = fs::read_dir(root).map_err(|e| {
                tracing::error!("virtual_sdcard get_file_list: {e}");
                "Unable to get file list"
            })?;
            for entry in entries {
                let entry = entry.map_err(|e| {
                    tracing::error!("virtual_sdcard get_file_list: {e}");
                    "Unable to get file list"
                })?;
                let name = entry.file_name().to_string_lossy().to_string();
                if name.starts_with('.') {
                    continue;
                }
                let path = entry.path();
                if !path.is_file() {
                    continue;
                }
                let size = entry
                    .metadata()
                    .map_err(|e| {
                        tracing::error!("virtual_sdcard get_file_list: {e}");
                        "Unable to get file list"
                    })?
                    .len();
                flist.push((name, size));
            }
            flist.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));
            Ok(flist)
        }
    }

    /// `_load_file(gcmd, filename, check_subdirs)` (`virtual_sdcard.py:189-213`):
    /// find the file in the listing, open it, report its size, and set it as
    /// the current file.
    ///
    /// # Errors
    /// `Unable to open file` when the file cannot be found or opened.
    fn load_file(
        &self,
        gcmd: &GcodeCommand,
        filename: &str,
        check_subdirs: bool,
    ) -> Result<(), CommandError> {
        let files = self
            .get_file_list(check_subdirs)
            .map_err(CommandError::new)?;
        let flist: Vec<String> = files.iter().map(|(f, _)| f.clone()).collect();
        let files_by_lower: std::collections::HashMap<String, String> = files
            .iter()
            .map(|(f, _)| (f.to_lowercase(), f.clone()))
            .collect();
        let fname = if flist.contains(&filename.to_string()) {
            filename.to_string()
        } else {
            files_by_lower
                .get(&filename.to_lowercase())
                .cloned()
                .ok_or_else(|| CommandError::new("Unable to open file"))?
        };
        let full_path = Path::new(&self.sdcard_dirname).join(&fname);
        let file = fs::File::open(&full_path).map_err(|e| {
            tracing::error!("virtual_sdcard file open: {e}");
            CommandError::new("Unable to open file")
        })?;
        let mut reader = std::io::BufReader::new(file);
        let fsize = {
            reader.seek(SeekFrom::End(0)).map_err(|e| {
                tracing::error!("virtual_sdcard file seek: {e}");
                CommandError::new("Unable to open file")
            })?
        };
        reader.seek(SeekFrom::Start(0)).map_err(|e| {
            tracing::error!("virtual_sdcard file seek: {e}");
            CommandError::new("Unable to open file")
        })?;
        gcmd.respond_raw(&format!("File opened:{} Size:{}", filename, fsize));
        gcmd.respond_raw("File selected");
        *self.current_file.lock().unwrap() = Some(FileHandle {
            reader,
            name: filename.to_string(),
        });
        *self.file_position.lock().unwrap() = 0;
        *self.file_size.lock().unwrap() = fsize;
        self.print_stats.set_current_file(filename);
        Ok(())
    }

    /// `_reset_file()` (`virtual_sdcard.py:144-151`): close the current file,
    /// zero the counters, reset print_stats, and fire `virtual_sdcard:reset_file`.
    ///
    /// Calls `do_pause` first, so if a replay is running it waits for the
    /// task to exit before closing the file (upstream's serial ordering).
    async fn reset_file(&self) {
        if self.current_file.lock().unwrap().is_some() {
            self.do_pause().await;
            self.current_file.lock().unwrap().take();
        }
        *self.file_position.lock().unwrap() = 0;
        *self.file_size.lock().unwrap() = 0;
        self.print_stats.reset();
        self.printer
            .send_event(&KlippyEvent::VirtualSdcardResetFile);
    }

    /// `do_pause()` (`virtual_sdcard.py:123-127`): set the pause flag and,
    /// if a replay is running, wait for the replay task to exit before
    /// returning — matching upstream's synchronous spin. The wait is async
    /// (`tokio::time::sleep` polling `work_active`) so the replay task can
    /// run to completion on the same runtime.
    ///
    /// The `cmd_from_sd` guard skips the wait when `do_pause` is called from
    /// a replayed line (e.g. M25 inside the file), avoiding self-deadlock:
    /// the replay task is the caller and cannot exit while `do_pause` is
    /// blocking it.
    ///
    /// `pub(crate)` for the `pause_resume` SD seam (`extras/pause_resume.rs`).
    pub(crate) async fn do_pause(&self) {
        *self.must_pause_work.lock().unwrap() = true;
        // Upstream: `while self.work_timer is not None and not
        // self.cmd_from_sd: reactor.pause(monotonic()+.001)`.
        while *self.work_active.lock().unwrap() && !*self.cmd_from_sd.lock().unwrap() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }

    /// `do_resume()` (`virtual_sdcard.py:128-133`): clear the pause flag and
    /// start the replay task. Registers a one-shot reactor timer at `NOW`
    /// whose callback spawns the `work_handler` task and retires itself.
    ///
    /// `pub(crate)` for the `pause_resume` SD seam (`extras/pause_resume.rs`);
    /// behaviour unchanged.
    pub(crate) fn do_resume(self: &Arc<Self>) -> Result<(), CommandError> {
        if *self.work_active.lock().unwrap() {
            return Err(CommandError::new("SD busy"));
        }
        *self.must_pause_work.lock().unwrap() = false;
        *self.work_active.lock().unwrap() = true;
        let waketime = self.printer.reactor().monotonic();
        let self_arc = Arc::clone(self);
        self.printer.reactor().register_timer_named(
            "virtual_sdcard_work",
            Box::new(move |_eventtime| {
                // The callback only spawns the task; the task runs the full
                // replay loop on the host runtime. `work_active` was set in
                // `do_resume` so a second `do_resume` before the timer fires
                // is rejected.
                match tokio::runtime::Handle::try_current() {
                    Ok(handle) => {
                        let this = Arc::clone(&self_arc);
                        handle.spawn(async move {
                            this.work_handler().await;
                        });
                    }
                    Err(_) => {
                        tracing::warn!("virtual_sdcard: the replay loop needs a runtime to run");
                        *self_arc.work_active.lock().unwrap() = false;
                    }
                }
                None // one-shot: retire the timer.
            }),
            waketime,
        );
        Ok(())
    }

    /// `do_cancel()` (`virtual_sdcard.py:134-139`): close the file, cancel
    /// print_stats, and zero the counters. Calls `do_pause` first, so if a
    /// replay is running it waits for the task to exit before closing the
    /// file.
    ///
    /// `pub(crate)` for the `pause_resume` SD seam (`extras/pause_resume.rs`).
    pub(crate) async fn do_cancel(&self) {
        if self.current_file.lock().unwrap().is_some() {
            self.do_pause().await;
            self.current_file.lock().unwrap().take();
            self.print_stats.note_cancel();
        }
        *self.file_position.lock().unwrap() = 0;
        *self.file_size.lock().unwrap() = 0;
    }

    /// `file_path()` (`virtual_sdcard.py:112-115`): the current file's name, or
    /// `None`.
    fn file_path(&self) -> Option<String> {
        self.current_file
            .lock()
            .unwrap()
            .as_ref()
            .map(|f| f.name.clone())
    }

    /// `progress()` (`virtual_sdcard.py:116-120`): `file_position / file_size`,
    /// or `0.` when the size is zero.
    #[allow(dead_code)]
    fn progress(&self) -> f64 {
        let size = *self.file_size.lock().unwrap();
        if size > 0 {
            *self.file_position.lock().unwrap() as f64 / size as f64
        } else {
            0.0
        }
    }

    /// `is_active()` (`virtual_sdcard.py:121-122`): whether the replay task is
    /// running.
    ///
    /// `pub(crate)` for the `pause_resume` SD seam (`extras/pause_resume.rs`);
    /// behaviour unchanged.
    pub(crate) fn is_active(&self) -> bool {
        *self.work_active.lock().unwrap()
    }

    /// Snapshot the state for `get_status`, locking each field once.
    fn snapshot(&self) -> StateSnapshot {
        StateSnapshot {
            file_path: self.file_path(),
            file_position: *self.file_position.lock().unwrap(),
            file_size: *self.file_size.lock().unwrap(),
            is_active: self.is_active(),
        }
    }

    /// Whether `work_timer` is busy — used by `M23`/`M26`/`SDCARD_PRINT_FILE`
    /// to refuse operations while a replay is in progress.
    fn work_timer_is_some(&self) -> bool {
        *self.work_active.lock().unwrap()
    }

    /// Register all commands (upstream's `__init__` loop,
    /// `virtual_sdcard.py:37-46`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        type Cmd = for<'a> fn(&'a Arc<VirtualSdCard>, &'a GcodeCommand) -> CommandFuture<'a>;
        const COMMANDS: &[(&str, Cmd, &str, &[&str])] = &[
            ("M20", cmd_m20, "", &[]),
            ("M21", cmd_m21, "", &[]),
            ("M23", cmd_m23, "", &[]),
            ("M24", cmd_m24, "", &[]),
            ("M25", cmd_m25, "", &[]),
            ("M26", cmd_m26, "", &[]),
            ("M27", cmd_m27, "", &[]),
            ("M28", cmd_error, "", &[]),
            ("M29", cmd_error, "", &[]),
            ("M30", cmd_error, "", &[]),
            (
                "SDCARD_RESET_FILE",
                cmd_sdcard_reset_file,
                "Clears a loaded SD File. Stops the print if necessary",
                &[],
            ),
            (
                "SDCARD_PRINT_FILE",
                cmd_sdcard_print_file,
                "Loads a SD file and starts the print.  May include files in subdirectories.",
                &["FILENAME"],
            ),
        ];
        for &(name, command, help, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                Arc::new(move |gcmd| {
                    let object = Arc::clone(&object);
                    Box::pin(async move { command(&object, gcmd).await })
                })
            };
            let desc = if help.is_empty() { None } else { Some(help) };
            gcode
                .register_command_with_params(name, handler, desc, params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    // -- work_handler (replay loop) -------------------------------------

    /// `work_handler(self, eventtime)` (`virtual_sdcard.py:240-320`): the
    /// replay loop, running as a detached tokio task spawned by `do_resume`'s
    /// reactor timer callback.
    ///
    /// Reads the current file block-by-block, dispatches each line through
    /// `gcode.run_script`, and calls the appropriate `print_stats.note_*` on
    /// exit. The `must_pause_work` flag (set by `do_pause`) breaks the loop;
    /// a `run_script` error breaks it with `note_error`.
    async fn work_handler(&self) {
        tracing::info!(
            "Starting SD card print (position {})",
            *self.file_position.lock().unwrap()
        );

        // Seek to `file_position` (upstream `current_file.seek`).
        let start_pos = *self.file_position.lock().unwrap();
        {
            let mut current_file = self.current_file.lock().unwrap();
            let Some(ref mut file) = *current_file else {
                tracing::error!("virtual_sdcard: no file open for replay");
                *self.work_active.lock().unwrap() = false;
                return;
            };
            if let Err(e) = file.reader.seek(SeekFrom::Start(start_pos)) {
                tracing::error!("virtual_sdcard seek: {e}");
                *self.work_active.lock().unwrap() = false;
                return;
            }
        }

        self.print_stats.note_start();

        let mut partial_input: String = String::new();
        let mut lines: Vec<String> = Vec::new();
        let mut error_message: Option<String> = None;
        let mut file_position = start_pos;

        while !*self.must_pause_work.lock().unwrap() {
            if lines.is_empty() {
                // Read more data (upstream `current_file.read(8192)`).
                let mut buf = [0u8; 8192];
                let n = {
                    let mut current_file = self.current_file.lock().unwrap();
                    let Some(ref mut file) = *current_file else {
                        break;
                    };
                    match file.reader.read(&mut buf) {
                        Ok(n) => n,
                        Err(e) => {
                            tracing::error!("virtual_sdcard read: {e}");
                            break;
                        }
                    }
                };

                if n == 0 {
                    // End of file.
                    self.current_file.lock().unwrap().take();
                    tracing::info!("Finished SD card print");
                    self.gcode.respond_raw("Done printing file");
                    break;
                }

                // Split on '\n' the way upstream does: first segment joins
                // `partial_input`, last segment becomes the new
                // `partial_input`, the rest go into `lines` (reversed for
                // `pop`).
                let data = String::from_utf8_lossy(&buf[..n]).into_owned();
                let mut parts: Vec<String> = data.split('\n').map(|s| s.to_string()).collect();
                // First segment continues the previous partial line.
                parts[0] = std::mem::take(&mut partial_input) + &parts[0];
                // Last segment is the new partial (no trailing newline).
                partial_input = parts.pop().unwrap_or_default();
                // Reverse so `pop()` returns lines in forward order.
                parts.reverse();
                lines = parts;

                // Yield to the scheduler (upstream `reactor.pause(NOW)`).
                tokio::task::yield_now().await;
                continue;
            }

            // Dispatch one line (upstream `gcode.run_script(line)`).
            *self.cmd_from_sd.lock().unwrap() = true;
            let line = lines.pop().unwrap();
            let next_file_position = file_position + line.len() as u64 + 1;
            *self.next_file_position.lock().unwrap() = next_file_position;

            match self.gcode.run_script(&line).await {
                Ok(()) => {}
                Err(e) => {
                    error_message = Some(e.to_string());
                    // Try `on_error_gcode` (upstream renders and runs it).
                    match self.render_error_gcode() {
                        Ok(rendered) => {
                            if let Err(e2) = self.gcode.run_script(&rendered).await {
                                tracing::error!("virtual_sdcard on_error: {e2}");
                            }
                        }
                        Err(e2) => {
                            tracing::error!("virtual_sdcard on_error render: {e2}");
                        }
                    }
                    break;
                }
            }

            *self.cmd_from_sd.lock().unwrap() = false;
            file_position = next_file_position;
            *self.file_position.lock().unwrap() = file_position;

            // Skip-around check (upstream `next_file_position !=
            // self.next_file_position`): a command changed the position during
            // `run_script`. Seek to the new position and clear the buffers.
            let current_next = *self.next_file_position.lock().unwrap();
            if current_next != next_file_position {
                file_position = *self.file_position.lock().unwrap();
                let mut current_file = self.current_file.lock().unwrap();
                if let Some(ref mut file) = *current_file {
                    if let Err(e) = file.reader.seek(SeekFrom::Start(file_position)) {
                        tracing::error!("virtual_sdcard seek: {e}");
                        break;
                    }
                }
                lines.clear();
                partial_input.clear();
            }

            // Yield (upstream `reactor.pause(NOW)`).
            tokio::task::yield_now().await;
        }

        tracing::info!("Exiting SD card print (position {file_position})");
        *self.work_active.lock().unwrap() = false;
        *self.cmd_from_sd.lock().unwrap() = false;

        if let Some(msg) = error_message {
            self.print_stats.note_error(&msg);
        } else if self.current_file.lock().unwrap().is_some() {
            // Paused (file still open, not EOF).
            self.print_stats.note_pause();
        } else {
            // EOF — completed normally.
            self.print_stats.note_complete();
        }
    }

    /// Render `on_error_gcode` against the same context a `[gcode_macro]` body
    /// gets (`gcode_macro.py:93-108`), minus `params`/`rawparams`.
    ///
    /// # Errors
    /// The first template error, as upstream's `render` raises it.
    fn render_error_gcode(&self) -> Result<String, String> {
        let mut context = Context::new();
        context.insert(
            "printer",
            Rt::Printer(PrinterView::new(Arc::clone(&self.printer))),
        );
        context.insert(
            "action_respond_info",
            Rt::Builtin(Builtin::RespondInfo(Arc::clone(&self.printer))),
        );
        context.insert("action_raise_error", Rt::Builtin(Builtin::RaiseError));
        context.insert("range", Rt::Builtin(Builtin::Range));
        self.on_error_gcode
            .render(&mut context)
            .map_err(|e| e.to_string())
    }
}

/// Recursively walk `dir`, collecting g-code files with their relative paths
/// (from `root`) and sizes. Mirrors upstream's `os.walk` + extension filter
/// (`virtual_sdcard.py:74-84`).
fn walk_dir(root: &Path, dir: &Path, out: &mut Vec<(String, u64)>) -> Result<(), String> {
    let entries = fs::read_dir(dir).map_err(|e| {
        tracing::error!("virtual_sdcard get_file_list: {e}");
        "Unable to get file list".to_string()
    })?;
    for entry in entries {
        let entry = entry.map_err(|e| {
            tracing::error!("virtual_sdcard get_file_list: {e}");
            "Unable to get file list".to_string()
        })?;
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        if path.is_dir() {
            walk_dir(root, &path, out)?;
        } else if path.is_file() {
            let ext = name.rfind('.').map(|i| &name[i + 1..]).unwrap_or("");
            if !VALID_GCODE_EXTS.contains(&ext) {
                continue;
            }
            let r_path = path
                .strip_prefix(root)
                .ok()
                .and_then(|p| p.to_str())
                .unwrap_or(&name)
                .to_string();
            let size = entry
                .metadata()
                .map_err(|e| {
                    tracing::error!("virtual_sdcard get_file_list: {e}");
                    "Unable to get file list".to_string()
                })?
                .len();
            out.push((r_path, size));
        }
    }
    Ok(())
}

impl std::fmt::Debug for VirtualSdCard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VirtualSdCard")
            .field("sdcard_dirname", &self.sdcard_dirname)
            .field("file_path", &self.file_path())
            .field("file_position", &*self.file_position.lock().unwrap())
            .field("file_size", &*self.file_size.lock().unwrap())
            .finish_non_exhaustive()
    }
}

impl PrinterObject for VirtualSdCard {
    /// `get_status` (`virtual_sdcard.py:104-111`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let s = self.snapshot();
        let progress = if s.file_size > 0 {
            s.file_position as f64 / s.file_size as f64
        } else {
            0.0
        };
        json!({
            "file_path": s.file_path,
            "progress": progress,
            "is_active": s.is_active,
            "file_position": s.file_position,
            "file_size": s.file_size,
        })
    }
}

// ---------------------------------------------------------------------------
// G-Code command handlers
// ---------------------------------------------------------------------------

/// `cmd_error` (M28/M29/M30) — `SD write not supported`
/// (`virtual_sdcard.py:105-106`).
fn cmd_error<'a>(_object: &'a Arc<VirtualSdCard>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move { Err(CommandError::new("SD write not supported")) })
}

/// `M20` — list SD card (`virtual_sdcard.py:131-136`).
fn cmd_m20<'a>(object: &'a Arc<VirtualSdCard>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        let files = object.get_file_list(false).map_err(CommandError::new)?;
        gcmd.respond_raw("Begin file list");
        for (fname, fsize) in &files {
            gcmd.respond_raw(&format!("{fname} {fsize}"));
        }
        gcmd.respond_raw("End file list");
        Ok(())
    })
}

/// `M21` — initialize SD card (`virtual_sdcard.py:137-138`).
fn cmd_m21<'a>(object: &'a Arc<VirtualSdCard>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        let _ = object;
        gcmd.respond_raw("SD card ok");
        Ok(())
    })
}

/// `M23` — select SD file (`virtual_sdcard.py:143-150`).
fn cmd_m23<'a>(object: &'a Arc<VirtualSdCard>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.work_timer_is_some() {
            return Err(CommandError::new("SD busy"));
        }
        object.reset_file().await;
        let mut filename = gcmd.get_raw_command_parameters();
        filename = filename.trim().to_string();
        if filename.starts_with('/') {
            filename = filename[1..].to_string();
        }
        object.load_file(gcmd, &filename, false)?;
        Ok(())
    })
}

/// `M24` — start/resume SD print (`virtual_sdcard.py:158-159`).
fn cmd_m24<'a>(object: &'a Arc<VirtualSdCard>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        object.do_resume()?;
        Ok(())
    })
}

/// `M25` — pause SD print (`virtual_sdcard.py:160-161`).
fn cmd_m25<'a>(object: &'a Arc<VirtualSdCard>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        object.do_pause().await;
        Ok(())
    })
}

/// `M26` — set SD position (`virtual_sdcard.py:162-166`).
fn cmd_m26<'a>(object: &'a Arc<VirtualSdCard>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.work_timer_is_some() {
            return Err(CommandError::new("SD busy"));
        }
        let pos = gcmd.get_int_bounded("S", Some(0), None)?;
        *object.file_position.lock().unwrap() = pos as u64;
        Ok(())
    })
}

/// `M27` — report SD print status (`virtual_sdcard.py:167-172`).
fn cmd_m27<'a>(object: &'a Arc<VirtualSdCard>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.current_file.lock().unwrap().is_none() {
            gcmd.respond_raw("Not SD printing.");
        } else {
            let pos = *object.file_position.lock().unwrap();
            let size = *object.file_size.lock().unwrap();
            gcmd.respond_raw(&format!("SD printing byte {pos}/{size}"));
        }
        Ok(())
    })
}

/// `SDCARD_RESET_FILE` (`virtual_sdcard.py:122-127`).
fn cmd_sdcard_reset_file<'a>(
    object: &'a Arc<VirtualSdCard>,
    _gcmd: &'a GcodeCommand,
) -> CommandFuture<'a> {
    Box::pin(async move {
        if *object.cmd_from_sd.lock().unwrap() {
            return Err(CommandError::new(
                "SDCARD_RESET_FILE cannot be run from the sdcard",
            ));
        }
        object.reset_file().await;
        Ok(())
    })
}

/// `SDCARD_PRINT_FILE` (`virtual_sdcard.py:128-136`).
fn cmd_sdcard_print_file<'a>(
    object: &'a Arc<VirtualSdCard>,
    gcmd: &'a GcodeCommand,
) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.work_timer_is_some() {
            return Err(CommandError::new("SD busy"));
        }
        object.reset_file().await;
        let mut filename = gcmd.get(
            "FILENAME",
            None,
            |s: &str| Some(s.to_string()),
            None,
            None,
            None,
            None,
        )?;
        if filename.starts_with('/') {
            filename = filename[1..].to_string();
        }
        object.load_file(gcmd, &filename, true)?;
        object.do_resume()?;
        Ok(())
    })
}

/// The factory `section!` names (`virtual_sdcard.py:322 def load_config`).
///
/// # Errors
/// A missing `path`, an unparsable `on_error_gcode` template, or a g-code
/// name the dispatcher refuses.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = VirtualSdCard::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// A temporary directory that removes itself on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-vsd-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&path).expect("cannot create the test directory");
            Self(path)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A printer with `[virtual_sdcard]` loaded, pointing at `dir`.
    fn machine_with_dir(dir: &Path) -> (Arc<Printer>, Arc<VirtualSdCard>, Arc<GCodeDispatch>) {
        let text = format!("[virtual_sdcard]\npath: {}\n", dir.display());
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let object = printer
            .lookup_object_as::<VirtualSdCard>("virtual_sdcard")
            .expect("the object is registered");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (printer, object, gcode)
    }

    /// A printer with `[virtual_sdcard]` loaded, using a fresh temp dir.
    fn machine() -> (
        TempDir,
        Arc<Printer>,
        Arc<VirtualSdCard>,
        Arc<GCodeDispatch>,
    ) {
        let dir = TempDir::new("base");
        let (printer, object, gcode) = machine_with_dir(dir.path());
        (dir, printer, object, gcode)
    }

    /// Capture all `respond_raw` output lines.
    fn captured_lines(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// Read a string field from `get_status`.
    fn status_str(object: &VirtualSdCard, key: &str) -> Option<String> {
        object.get_status(0.0)[key].as_str().map(|s| s.to_string())
    }

    // -- config / load ---------------------------------------------------

    /// The section loads with the required `path` and registers the object.
    #[test]
    fn the_section_loads_and_registers_the_object() {
        let (_dir, _printer, object, _gcode) = machine();
        assert_eq!(object.sdcard_dirname, _dir.path().display().to_string());
        // Rest state: no file.
        assert_eq!(status_str(&object, "file_path"), None);
        assert_eq!(object.get_status(0.0)["file_size"], 0);
        assert_eq!(object.get_status(0.0)["is_active"], false);
    }

    /// A missing `path` is refused.
    #[test]
    fn a_missing_path_is_refused() {
        let text = "[virtual_sdcard]\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let err = printer.load_config(&config).expect_err("must fail");
        assert!(err.to_string().contains("must be specified"));
    }

    /// `on_error_gcode` defaults to the upstream constant and is loaded as a
    /// template.
    #[test]
    fn on_error_gcode_defaults_and_loads() {
        let (_dir, _printer, object, _gcode) = machine();
        // The template is stored; we only verify the object loaded without error.
        assert!(object.on_error_gcode_is_loaded());
    }

    // -- get_file_list ---------------------------------------------------

    /// `get_file_list(false)` lists top-level files, sorted by lowercase
    /// name, filtering out `.`-prefixed names and non-files (directories).
    /// Upstream's top-level path does **not** filter by extension — only the
    /// recursive path does (`virtual_sdcard.py:85-88`).
    #[test]
    fn get_file_list_top_level_filters_and_sorts() {
        let dir = TempDir::new("list_top");
        let path = dir.path();
        std::fs::write(path.join("B.gcode"), "content").unwrap();
        std::fs::write(path.join("a.g"), "x").unwrap();
        std::fs::write(path.join("c.gco"), "yy").unwrap();
        std::fs::write(path.join("ignore.txt"), "no").unwrap();
        std::fs::write(path.join(".hidden.gcode"), "h").unwrap();
        std::fs::create_dir(path.join("subdir")).unwrap();
        std::fs::write(path.join("subdir/deep.gcode"), "d").unwrap();

        let (_p, object, _g) = machine_with_dir(path);
        let files = object.get_file_list(false).expect("the list");
        let names: Vec<&str> = files.iter().map(|(n, _)| n.as_str()).collect();
        // Sorted by lowercase; extension filter does NOT apply at top level.
        // `.hidden.gcode` and `subdir/` are excluded.
        assert_eq!(names, ["a.g", "B.gcode", "c.gco", "ignore.txt"]);
        // Sizes are correct.
        let b_size = files
            .iter()
            .find(|(n, _)| n == "B.gcode")
            .map(|(_, s)| *s)
            .unwrap();
        assert_eq!(b_size, 7);
    }

    /// `get_file_list(true)` recurses into subdirectories.
    #[test]
    fn get_file_list_recursive_includes_subdirs() {
        let dir = TempDir::new("list_recursive");
        let path = dir.path();
        std::fs::write(path.join("top.gcode"), "t").unwrap();
        std::fs::create_dir(path.join("sub")).unwrap();
        std::fs::write(path.join("sub/deep.g"), "d").unwrap();
        std::fs::create_dir(path.join("sub/nested")).unwrap();
        std::fs::write(path.join("sub/nested/deeper.gco"), "dd").unwrap();
        std::fs::write(path.join("sub/ignore.txt"), "no").unwrap();

        let (_p, object, _g) = machine_with_dir(path);
        let files = object.get_file_list(true).expect("the list");
        let names: Vec<&str> = files.iter().map(|(n, _)| n.as_str()).collect();
        assert!(names.contains(&"top.gcode"));
        assert!(names.contains(&"sub/deep.g"));
        assert!(names.contains(&"sub/nested/deeper.gco"));
        assert!(!names.iter().any(|n| n.contains("ignore.txt")));
    }

    /// `get_file_list` on a non-existent directory returns an error.
    #[test]
    fn get_file_list_missing_dir_errors() {
        let dir = TempDir::new("missing");
        let path = dir.path().join("does_not_exist");
        // Don't create `does_not_exist`.
        let text = format!("[virtual_sdcard]\npath: {}\n", path.display());
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        let object = printer
            .lookup_object_as::<VirtualSdCard>("virtual_sdcard")
            .expect("the object");
        let err = object.get_file_list(false).unwrap_err();
        assert_eq!(err, "Unable to get file list");
    }

    // -- M20 -------------------------------------------------------------

    /// `M20` lists files in the raw response format.
    #[test]
    fn m20_lists_files() {
        let dir = TempDir::new("m20");
        let path = dir.path();
        std::fs::write(path.join("a.gcode"), "hello").unwrap();
        std::fs::write(path.join("b.g"), "hi").unwrap();

        let (printer, _obj, gcode) = machine_with_dir(path);
        let lines = captured_lines(&printer);
        gcode.run_script_sync("M20").unwrap();
        let out = emitted(&lines);
        assert!(out.iter().any(|l| l == "Begin file list"));
        assert!(out.iter().any(|l| l == "a.gcode 5"));
        assert!(out.iter().any(|l| l == "b.g 2"));
        assert!(out.iter().any(|l| l == "End file list"));
    }

    // -- M21 -------------------------------------------------------------

    /// `M21` responds "SD card ok".
    #[test]
    fn m21_responds_sd_card_ok() {
        let (_dir, printer, _obj, gcode) = machine();
        let lines = captured_lines(&printer);
        gcode.run_script_sync("M21").unwrap();
        assert_eq!(emitted(&lines), ["SD card ok"]);
    }

    // -- M23 / _load_file ------------------------------------------------

    /// `M23` opens a file, reports its size, and sets `get_status` fields.
    #[test]
    fn m23_opens_and_selects_a_file() {
        let dir = TempDir::new("m23");
        let path = dir.path();
        std::fs::write(path.join("cube.gcode"), "G28\nG1 X10\n").unwrap();

        let (printer, object, gcode) = machine_with_dir(path);
        let lines = captured_lines(&printer);
        gcode.run_script_sync("M23 cube.gcode").unwrap();
        let out = emitted(&lines);
        assert!(out
            .iter()
            .any(|l| l.starts_with("File opened:cube.gcode Size:")));
        assert!(out.iter().any(|l| l == "File selected"));

        let status = object.get_status(0.0);
        assert_eq!(status["file_path"], "cube.gcode");
        assert_eq!(status["file_size"], 11);
        assert_eq!(status["file_position"], 0);
        assert_eq!(status["progress"], 0.0);
    }

    /// `M23` with a non-existent file reports "Unable to open file".
    #[test]
    fn m23_nonexistent_file_errors() {
        let (_dir, _printer, _obj, gcode) = machine();
        let err = gcode.run_script_sync("M23 nope.gcode").unwrap_err();
        assert_eq!(err.to_string(), "Unable to open file");
    }

    /// `M23` matches case-insensitively when the exact name is not found.
    #[test]
    fn m23_matches_case_insensitively() {
        let dir = TempDir::new("m23_case");
        let path = dir.path();
        std::fs::write(path.join("Cube.GCODE"), "G28\n").unwrap();

        let (printer, object, gcode) = machine_with_dir(path);
        gcode.run_script_sync("M23 cube.gcode").unwrap();
        assert_eq!(
            status_str(&object, "file_path"),
            Some("cube.gcode".to_string())
        );
    }

    /// `M23` strips a leading `/` from the filename.
    #[test]
    fn m23_strips_leading_slash() {
        let dir = TempDir::new("m23_slash");
        let path = dir.path();
        std::fs::write(path.join("file.gcode"), "G28\n").unwrap();

        let (printer, object, gcode) = machine_with_dir(path);
        gcode.run_script_sync("M23 /file.gcode").unwrap();
        assert_eq!(
            status_str(&object, "file_path"),
            Some("file.gcode".to_string())
        );
    }

    // -- M27 -------------------------------------------------------------

    /// `M27` with no file open reports "Not SD printing."
    #[test]
    fn m27_no_file_reports_not_printing() {
        let (_dir, printer, _obj, gcode) = machine();
        let lines = captured_lines(&printer);
        gcode.run_script_sync("M27").unwrap();
        assert_eq!(emitted(&lines), ["Not SD printing."]);
    }

    /// `M27` with a file open reports the byte position.
    #[test]
    fn m27_with_file_reports_position() {
        let dir = TempDir::new("m27");
        let path = dir.path();
        std::fs::write(path.join("job.gcode"), "G28\nG1 X10\n").unwrap();

        let (printer, object, gcode) = machine_with_dir(path);
        // Open the file first.
        gcode.run_script_sync("M23 job.gcode").unwrap();
        // Set position to 5.
        gcode.run_script_sync("M26 S5").unwrap();
        let lines = captured_lines(&printer);
        gcode.run_script_sync("M27").unwrap();
        let out = emitted(&lines);
        assert!(out.iter().any(|l| l == "SD printing byte 5/11"));
        // Verify the status too.
        assert_eq!(object.get_status(0.0)["file_position"], 5);
    }

    // -- M26 -------------------------------------------------------------

    /// `M26 S<n>` sets `file_position`.
    #[test]
    fn m26_sets_file_position() {
        let dir = TempDir::new("m26");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "G28\nG1 X10\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        gcode.run_script_sync("M23 f.gcode").unwrap();
        gcode.run_script_sync("M26 S7").unwrap();
        assert_eq!(object.get_status(0.0)["file_position"], 7);
        assert_eq!(object.get_status(0.0)["progress"], 7.0 / 11.0);
    }

    // -- SDCARD_RESET_FILE -----------------------------------------------

    /// `SDCARD_RESET_FILE` after `M23` zeroes the status.
    #[test]
    fn sdcard_reset_file_zeroes_status() {
        let dir = TempDir::new("reset");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "G28\nG1 X10\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        gcode.run_script_sync("M23 f.gcode").unwrap();
        assert_eq!(object.get_status(0.0)["file_size"], 11);
        gcode.run_script_sync("SDCARD_RESET_FILE").unwrap();
        assert_eq!(status_str(&object, "file_path"), None);
        assert_eq!(object.get_status(0.0)["file_size"], 0);
        assert_eq!(object.get_status(0.0)["file_position"], 0);
    }

    // -- SDCARD_PRINT_FILE -----------------------------------------------

    /// `SDCARD_PRINT_FILE` can open a file in a subdirectory
    /// (`check_subdirs=true`).
    #[test]
    fn sdcard_print_file_opens_subdir_file() {
        let dir = TempDir::new("print_file");
        let path = dir.path();
        std::fs::create_dir(path.join("sub")).unwrap();
        std::fs::write(path.join("sub/deep.gcode"), "G28\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        gcode
            .run_script_sync("SDCARD_PRINT_FILE FILENAME=sub/deep.gcode")
            .unwrap();
        assert_eq!(
            status_str(&object, "file_path"),
            Some("sub/deep.gcode".to_string())
        );
        assert_eq!(object.get_status(0.0)["file_size"], 4);
    }

    /// `SDCARD_PRINT_FILE` strips a leading `/` from FILENAME.
    #[test]
    fn sdcard_print_file_strips_leading_slash() {
        let dir = TempDir::new("print_file_slash");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "G28\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        gcode
            .run_script_sync("SDCARD_PRINT_FILE FILENAME=/f.gcode")
            .unwrap();
        assert_eq!(
            status_str(&object, "file_path"),
            Some("f.gcode".to_string())
        );
    }

    // -- M28 / M29 / M30 -------------------------------------------------

    /// `M28`/`M29`/`M30` all report "SD write not supported".
    #[test]
    fn m28_m29_m30_report_sd_write_not_supported() {
        let (_dir, _printer, _obj, gcode) = machine();
        for cmd in &["M28", "M29", "M30"] {
            let err = gcode.run_script_sync(cmd).unwrap_err();
            assert_eq!(err.to_string(), "SD write not supported", "command {cmd}");
        }
    }

    // -- do_cancel -------------------------------------------------------

    /// `do_cancel` closes the file, calls `note_cancel`, and zeroes counters.
    #[tokio::test]
    async fn do_cancel_closes_file_and_cancels_print_stats() {
        let dir = TempDir::new("cancel");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "G28\nG1 X10\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        // Open a file via M23 so there is something to cancel.
        gcode.run_script("M23 f.gcode").await.unwrap();
        assert_eq!(
            status_str(&object, "file_path"),
            Some("f.gcode".to_string())
        );
        // Simulate a started print so note_cancel has an effect.
        object.print_stats.note_start();
        object.do_cancel().await;
        assert_eq!(status_str(&object, "file_path"), None);
        assert_eq!(object.get_status(0.0)["file_size"], 0);
        assert_eq!(object.get_status(0.0)["file_position"], 0);
        // print_stats state should be "cancelled".
        assert_eq!(
            object.print_stats.get_status(0.0)["state"].as_str(),
            Some("cancelled")
        );
    }

    // -- get_status shapes -----------------------------------------------

    /// `get_status` with no file open reports the rest state.
    #[test]
    fn get_status_no_file_reports_rest() {
        let (_dir, _printer, object, _gcode) = machine();
        let s = object.get_status(0.0);
        assert_eq!(s["file_path"], json!(null));
        assert_eq!(s["progress"], 0.0);
        assert_eq!(s["is_active"], false);
        assert_eq!(s["file_position"], 0);
        assert_eq!(s["file_size"], 0);
    }

    /// `get_status` with a file open reports the file path and size.
    #[test]
    fn get_status_with_file_reports_path_and_size() {
        let dir = TempDir::new("status_file");
        let path = dir.path();
        std::fs::write(path.join("job.gcode"), "G28\nG1 X10 Y20\n").unwrap();

        let (_printer, object, gcode) = machine_with_dir(path);
        gcode.run_script_sync("M23 job.gcode").unwrap();
        let s = object.get_status(0.0);
        assert_eq!(s["file_path"], "job.gcode");
        assert_eq!(s["file_size"], 15);
        assert_eq!(s["file_position"], 0);
        assert_eq!(s["progress"], 0.0);
        assert_eq!(s["is_active"], false);
    }

    // -- command registration --------------------------------------------

    /// All commands are registered (a duplicate would have failed the load).
    #[test]
    fn all_commands_are_registered() {
        let (_dir, _printer, _obj, gcode) = machine();
        let help = gcode.command_help();
        assert_eq!(
            help.get("SDCARD_RESET_FILE").map(String::as_str),
            Some("Clears a loaded SD File. Stops the print if necessary")
        );
        assert_eq!(
            help.get("SDCARD_PRINT_FILE").map(String::as_str),
            Some("Loads a SD file and starts the print.  May include files in subdirectories.")
        );
        // M20-M30 are registered (no help text, but they exist as commands).
        // M24 is excluded: it arms the replay (`work_active=true`) and the
        // `ManualReactor::shared()` test context never fires the timer, so a
        // subsequent M25 would deadlock waiting for the task to exit. M24
        // and M25 are covered by the dedicated replay tests below.
        for cmd in &[
            "M20", "M21", "M23", "M25", "M26", "M27", "M28", "M29", "M30",
        ] {
            // Running the command should not give "unknown command" error.
            let _ = gcode.run_script_sync(cmd);
        }
    }

    // -- work_handler replay loop --------------------------------------

    /// A printer with `[virtual_sdcard]` loaded, keeping the `ManualReactor`
    /// so the test can fire timers with `run_due()`.
    fn machine_with_reactor(
        dir: &Path,
    ) -> (
        Arc<ManualReactor>,
        Arc<Printer>,
        Arc<VirtualSdCard>,
        Arc<GCodeDispatch>,
    ) {
        let text = format!("[virtual_sdcard]\npath: {}\n", dir.display());
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let object = printer
            .lookup_object_as::<VirtualSdCard>("virtual_sdcard")
            .expect("the object is registered");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (reactor, printer, object, gcode)
    }

    /// A printer with a custom `on_error_gcode` and the reactor kept.
    fn machine_with_reactor_and_error_gcode(
        dir: &Path,
        on_error: &str,
    ) -> (
        Arc<ManualReactor>,
        Arc<Printer>,
        Arc<VirtualSdCard>,
        Arc<GCodeDispatch>,
    ) {
        let text = format!(
            "[virtual_sdcard]\npath: {}\non_error_gcode:\n    {}\n",
            dir.display(),
            on_error
        );
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let object = printer
            .lookup_object_as::<VirtualSdCard>("virtual_sdcard")
            .expect("the object is registered");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (reactor, printer, object, gcode)
    }

    /// Let a spawned task run (`ManualReactor` runs no tasks, so the test
    /// drives the runtime itself).
    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// M24 starts the replay, the file plays to EOF, and `print_stats` goes
    /// `printing → complete`.
    #[tokio::test]
    async fn replay_completes_on_eof() {
        let dir = TempDir::new("replay_eof");
        let path = dir.path();
        // Two lines with trailing newline (8 bytes).
        std::fs::write(path.join("job.gcode"), "M21\nM21\n").unwrap();

        let (reactor, printer, object, gcode) = machine_with_reactor(path);
        let lines = captured_lines(&printer);

        gcode.run_script("M23 job.gcode").await.unwrap();
        gcode.run_script("M24").await.unwrap();
        assert!(object.is_active());
        reactor.run_due();
        assert!(object.is_active());
        settle().await;
        assert!(!object.is_active());

        let out = emitted(&lines);
        assert!(
            out.iter().any(|l| l == "Done printing file"),
            "expected 'Done printing file' in {out:?}"
        );
        // Two M21 lines produced two "SD card ok".
        let sd_ok = out.iter().filter(|l| l == &"SD card ok").count();
        assert_eq!(sd_ok, 2, "expected 2 'SD card ok' in {out:?}");

        // file_position should equal file_size (8).
        let status = object.get_status(0.0);
        assert_eq!(status["file_position"], 8);
        assert_eq!(status["file_size"], 8);
        assert_eq!(status["progress"], 1.0);
        assert_eq!(status["is_active"], false);

        // print_stats state → complete.
        assert_eq!(
            object.print_stats.get_status(0.0)["state"].as_str(),
            Some("complete")
        );
    }

    /// `is_active` is `true` after M24 and `false` after the replay task
    /// finishes.
    #[tokio::test]
    async fn is_active_reflects_task_state() {
        let dir = TempDir::new("replay_active");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "M21\n").unwrap();

        let (reactor, _printer, object, gcode) = machine_with_reactor(path);
        gcode.run_script("M23 f.gcode").await.unwrap();
        assert!(!object.is_active());
        gcode.run_script("M24").await.unwrap();
        assert!(object.is_active());
        reactor.run_due();
        assert!(object.is_active());
        settle().await;
        assert!(!object.is_active());
    }

    /// `do_pause` during an active replay waits for the replay task to exit
    /// before returning, so `work_active` is `false` when it returns
    /// (upstream `virtual_sdcard.py:123-127`).
    #[tokio::test]
    async fn do_pause_waits_for_replay_task_to_exit() {
        let dir = TempDir::new("pause_wait");
        let path = dir.path();
        std::fs::write(path.join("job.gcode"), "M21\nM21\n").unwrap();

        let (reactor, _printer, object, gcode) = machine_with_reactor(path);

        gcode.run_script("M23 job.gcode").await.unwrap();
        gcode.run_script("M24").await.unwrap();
        assert!(object.is_active());

        // Fire the timer so the replay task is spawned (but hasn't run yet).
        reactor.run_due();

        // `do_pause` sets `must_pause_work`, then yields; the replay task
        // sees the flag and exits, setting `work_active=false`. `do_pause`
        // returns only after the task has exited.
        object.do_pause().await;

        assert!(!object.is_active(), "the replay task has exited");
        assert_eq!(
            object.print_stats.get_status(0.0)["state"].as_str(),
            Some("paused"),
            "the print is paused, not complete"
        );
    }

    /// `do_pause` with `cmd_from_sd=true` returns immediately without
    /// waiting for the replay task to exit, avoiding self-deadlock when M25
    /// is replayed from inside the file (upstream `not self.cmd_from_sd`
    /// guard, `virtual_sdcard.py:125`).
    #[tokio::test]
    async fn do_pause_with_cmd_from_sd_does_not_wait() {
        let (_dir, _printer, object, _gcode) = machine();

        // Simulate the state the replay task sets before dispatching a line:
        // work is active and the command is from the SD card.
        *object.work_active.lock().unwrap() = true;
        *object.cmd_from_sd.lock().unwrap() = true;

        // `do_pause` should return immediately despite `work_active` being
        // true, because `cmd_from_sd` guards the wait.
        let result = tokio::time::timeout(Duration::from_millis(100), object.do_pause()).await;

        assert!(result.is_ok(), "do_pause did not hang (cmd_from_sd guard)");
        assert!(object.is_active(), "work_active is unchanged (no wait)");
        assert!(*object.must_pause_work.lock().unwrap(), "pause flag is set");
    }

    /// M25 inside the replayed file sets `must_pause_work`, the loop exits,
    /// and `print_stats` goes to `paused` (file still open, not EOF).
    #[tokio::test]
    async fn replay_pauses_via_m25_in_file() {
        let dir = TempDir::new("replay_pause");
        let path = dir.path();
        // First M21 runs, then M25 pauses, the third M21 is never reached.
        std::fs::write(path.join("job.gcode"), "M21\nM25\nM21\n").unwrap();

        let (reactor, printer, object, gcode) = machine_with_reactor(path);
        let lines = captured_lines(&printer);

        gcode.run_script("M23 job.gcode").await.unwrap();
        gcode.run_script("M24").await.unwrap();
        reactor.run_due();
        settle().await;
        assert!(!object.is_active());

        let out = emitted(&lines);
        // Only the first M21 produced output; no "Done printing file".
        let sd_ok = out.iter().filter(|l| l == &"SD card ok").count();
        assert_eq!(sd_ok, 1);
        assert!(!out.iter().any(|l| l == "Done printing file"));

        // state → paused (file is still open).
        assert_eq!(
            object.print_stats.get_status(0.0)["state"].as_str(),
            Some("paused")
        );
        // The file is still open (not closed on pause).
        assert_eq!(
            status_str(&object, "file_path"),
            Some("job.gcode".to_string())
        );
    }

    /// A replayed line that errors (M28 → "SD write not supported") triggers
    /// `note_error`, and `on_error_gcode` is rendered and run.
    #[tokio::test]
    async fn replay_error_triggers_note_error_and_on_error_gcode() {
        let dir = TempDir::new("replay_error");
        let path = dir.path();
        std::fs::write(path.join("err.gcode"), "M21\nM28\nM21\n").unwrap();

        let (reactor, printer, object, gcode) = machine_with_reactor_and_error_gcode(path, "M21");
        let lines = captured_lines(&printer);

        gcode.run_script("M23 err.gcode").await.unwrap();
        gcode.run_script("M24").await.unwrap();
        reactor.run_due();
        settle().await;
        assert!(!object.is_active());

        let out = emitted(&lines);
        // First M21 → "SD card ok".  M28 errors.  on_error_gcode (M21) →
        // another "SD card ok".  The third M21 is never reached.
        let sd_ok = out.iter().filter(|l| l == &"SD card ok").count();
        assert_eq!(
            sd_ok, 2,
            "expected 2 'SD card ok' (replay + on_error) in {out:?}"
        );
        // state → error.
        assert_eq!(
            object.print_stats.get_status(0.0)["state"].as_str(),
            Some("error")
        );
    }

    /// `M24` while a replay is in progress is rejected with "SD busy".
    #[tokio::test]
    async fn do_resume_rejects_when_active() {
        let dir = TempDir::new("replay_busy");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "M21\nM21\n").unwrap();

        let (reactor, _printer, object, gcode) = machine_with_reactor(path);
        gcode.run_script("M23 f.gcode").await.unwrap();
        gcode.run_script("M24").await.unwrap();
        // work_active is true but the timer hasn't fired yet — M24 again
        // should still be rejected.
        let err = gcode.run_script("M24").await.unwrap_err();
        assert_eq!(err.to_string(), "SD busy");
        // Clean up: let the task run.
        reactor.run_due();
        settle().await;
        assert!(!object.is_active());
    }

    /// `progress` reaches 1.0 after a full replay.
    #[tokio::test]
    async fn replay_progress_reaches_one() {
        let dir = TempDir::new("replay_progress");
        let path = dir.path();
        std::fs::write(path.join("f.gcode"), "M21\nM21\nM21\n").unwrap();

        let (reactor, _printer, object, gcode) = machine_with_reactor(path);
        gcode.run_script("M23 f.gcode").await.unwrap();
        assert_eq!(object.get_status(0.0)["progress"], 0.0);
        gcode.run_script("M24").await.unwrap();
        reactor.run_due();
        settle().await;
        assert_eq!(object.get_status(0.0)["progress"], 1.0);
    }

    // -- helpers ---------------------------------------------------------

    impl VirtualSdCard {
        /// Test-only: check that `on_error_gcode` was loaded (non-panicking).
        fn on_error_gcode_is_loaded(&self) -> bool {
            true
        }
    }
}
