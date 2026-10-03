//! The G-Code dispatcher: the command table and the script runner.
//!
//! Everything a user can type — over the API, from a macro, from the console —
//! goes through here. It parses a line into a command name and a parameter map,
//! finds the handler registered for that name, runs it, and turns whatever the
//! handler reports into output lines.
//!
//! Upstream is `klippy/gcode.py`. The shape is the same:
//!
//! * [`GCodeDispatch`] is the printer object `gcode`; other modules register
//!   into it (`register_command`, `register_mux_command`);
//! * [`GcodeCommand`] is the parsed command handed to a handler, with the
//!   `get_*` accessors that report a missing or malformed parameter as a
//!   [`CommandError`];
//! * [`GCodeDispatch::run_script`] is the entry point: split on newlines, run
//!   each line, stop at the first error.
//!
//! # Traditional and extended commands
//!
//! A *traditional* command is a letter followed by digits (`M110`, `G1`); its
//! parameters are `S200`, `X10.5` — a single letter and a value. Anything else
//! registered is an *extended* command (`SET_PIN`, `BED_MESH_CALIBRATE`); its
//! parameters are `KEY=VALUE`, shell-quoted. Upstream re-parses the raw text for
//! extended commands (`_get_extended_params`), and so does this port — see
//! [`parse_extended`].
//!
//! # What is not here
//!
//! * **Motion.** `G0`/`G1`/`G28`/… are registered by the toolhead; this module
//!   is only the dispatcher, so it does not depend on one.
//! * **The `ok` acknowledgement.** Upstream's `ack()` belongs to the file-output
//!   and debug-input protocols (`GCodeIO`), which this host does not have yet.
//! * **`gcode:command_error`.** Upstream fires that event on a handler error;
//!   we do too, on the `KlippyEvent` bus (an open vocabulary with an `Unknown`
//!   fallback, see the event system doc), not a closed event set.
//! * **`run_script`'s mutex.** Upstream serialises scripts on the reactor's
//!   mutex. Here a script runs to completion on the calling task; a second
//!   caller would interleave only at awaits, and nothing in a handler awaits.

use std::collections::HashMap;
use std::fmt;
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Map, Value};
use tracing::{error, info, warn};

use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name other modules use to find the dispatcher.
pub const GCODE_OBJECT: &str = "gcode";

/// What a command handler returns: a boxed future, so handlers may `.await`
/// (homing, temperature waits, probes) without parking a runtime worker.
pub type CommandFuture<'a> = Pin<Box<dyn Future<Output = Result<(), CommandError>> + Send + 'a>>;

/// A command handler: it reads its parameters from the command and either
/// succeeds or reports why it could not.
///
/// The future borrows the [`GcodeCommand`], which is `Sync` (its ack flag is an
/// `AtomicBool`), so the returned future is `Send` and can run on any task.
///
/// `Arc` rather than `Box` because a mux command's per-value handlers are stored
/// beside the dispatcher's own table and looked up again.
pub type CommandHandler = Arc<dyn for<'a> Fn(&'a GcodeCommand) -> CommandFuture<'a> + Send + Sync>;

/// Wrap a synchronous body as a [`CommandHandler`].
///
/// Most handlers never await; they use this so their bodies stay ordinary
/// `Fn(&GcodeCommand) -> Result<…>` functions.
pub fn sync<F>(body: F) -> CommandHandler
where
    F: Fn(&GcodeCommand) -> Result<(), CommandError> + Send + Sync + 'static,
{
    // The body runs on the **first poll**, not when the future is built, so a
    // panic is caught by `catch_unwind_future` like an async handler's would be.
    let body = Arc::new(body);
    Arc::new(move |gcmd| {
        let body = Arc::clone(&body);
        Box::pin(async move { body(gcmd) })
    })
}

/// A sink for the lines the dispatcher emits.
///
/// Implemented for any `Fn(&str)`, which is all a simple sink is. A sink that
/// belongs to a client also reports when that client goes away, so the
/// dispatcher can drop it: `gcode/subscribe_output` is the first of those.
pub trait OutputHandler: Send + Sync {
    /// Hand one line to the sink.
    fn emit(&self, line: &str);

    /// Whether this sink is gone and should be dropped.
    fn is_closed(&self) -> bool {
        false
    }
}

impl<F: Fn(&str) + Send + Sync> OutputHandler for F {
    fn emit(&self, line: &str) {
        self(line)
    }
}

/// A G-Code command failed.
///
/// The message is what the user sees, and upstream's wording is kept: it names
/// the whole command line, so a client that sent twelve commands knows which one
/// failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommandError {
    message: String,
}

impl CommandError {
    /// A command error with the message the user will see.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// The message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for CommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for CommandError {}

// ===========================================================================
// GcodeCommand
// ===========================================================================

/// Parse an integer parameter; `None` when it is not one.
fn parse_int(value: &str) -> Option<i64> {
    value.parse::<i64>().ok()
}

/// Parse a float parameter; `None` when it is not one.
///
/// `pub(crate)` so an extra that needs a combination the shorthand getters do
/// not spell (a default *and* a `minval`, say — upstream's
/// `get_float('S', 255., minval=0.)`) can pass it to
/// [`GcodeCommand::get`], which is what those shorthands wrap.
pub(crate) fn parse_float(value: &str) -> Option<f64> {
    value.parse::<f64>().ok()
}

/// One parsed command, as handed to its handler.
pub struct GcodeCommand {
    dispatch: Arc<Inner>,
    command: String,
    commandline: String,
    params: HashMap<String, String>,
    /// Whether this line still wants an `ok` ack.
    ///
    /// Upstream's `need_ack` (`klippy/gcode.py:23`): true for the file/serial
    /// input protocol, false for an API `gcode/script` line. [`GcodeCommand::ack`]
    /// clears it, so a handler that acks itself is not acked again by the
    /// trailing `gcmd.ack()` of `_process_commands`. An `AtomicBool` rather than
    /// a `Cell` so `&GcodeCommand` is `Send` and a handler's future can await.
    need_ack: AtomicBool,
}

impl GcodeCommand {
    /// The command name (`M110`, `SET_PIN`), without parameters.
    pub fn command(&self) -> &str {
        &self.command
    }

    /// The line as the client sent it, trimmed (comment included).
    pub fn commandline(&self) -> &str {
        &self.commandline
    }

    /// Every parameter, by name (upstream's `get_command_parameters`).
    pub fn get_command_parameters(&self) -> &HashMap<String, String> {
        &self.params
    }

    /// The text after the command name, as the client typed it.
    ///
    /// Upstream's `get_raw_command_parameters` (`klippy/gcode.py:40-51`): on a
    /// line-numbered line the leading `N<digits>` and a trailing
    /// `*<checksum>` are dropped; otherwise the text is returned as written.
    pub fn get_raw_command_parameters(&self) -> String {
        let command = self.command.as_str();
        let origline = self.commandline.as_str();
        let mut param_start = command.len();
        let mut param_end = origline.len();
        let head = origline.get(..param_start).unwrap_or(origline);
        if !head.eq_ignore_ascii_case(command) {
            // A line number (or a differently-cased command) precedes it: find
            // the command itself and drop a trailing checksum.
            match origline
                .to_ascii_uppercase()
                .find(&command.to_ascii_uppercase())
            {
                Some(pos) => param_start += pos,
                None => return String::new(),
            }
            if let Some(star) = origline.rfind('*') {
                let checksum = &origline[star + 1..];
                if !checksum.is_empty() && checksum.bytes().all(|b| b.is_ascii_digit()) {
                    param_end = star;
                }
            }
        }
        if origline
            .as_bytes()
            .get(param_start)
            .is_some_and(u8::is_ascii_whitespace)
        {
            param_start += 1;
        }
        origline
            .get(param_start..param_end)
            .unwrap_or("")
            .to_string()
    }

    /// A required string parameter.
    ///
    /// # Errors
    /// Returns [`CommandError`] naming the line when the parameter is absent.
    pub fn get_str(&self, name: &str) -> Result<String, CommandError> {
        self.params
            .get(name)
            .cloned()
            .ok_or_else(|| self.missing(name))
    }

    /// A string parameter, or `default` when it is absent.
    pub fn get_str_default(&self, name: &str, default: &str) -> String {
        self.params
            .get(name)
            .cloned()
            .unwrap_or_else(|| default.to_string())
    }

    /// A parameter parsed by `parse`, with optional bounds.
    ///
    /// Upstream's `GCodeCommand.get` (`klippy/gcode.py:65-90`): `default` is
    /// `None` when the parameter is required, and the bound messages keep
    /// upstream's wording ("must have minimum of …", "must be above …").
    ///
    /// # Errors
    /// [`CommandError`] when the parameter is absent and has no default, cannot
    /// be parsed, or falls outside a bound.
    // The bounds mirror upstream's `GCodeCommand.get` one for one; folding them
    // into a struct would hide the wording the errors keep.
    #[allow(clippy::too_many_arguments)]
    pub fn get<T>(
        &self,
        name: &str,
        default: Option<T>,
        parse: impl FnOnce(&str) -> Option<T>,
        minval: Option<T>,
        maxval: Option<T>,
        above: Option<T>,
        below: Option<T>,
    ) -> Result<T, CommandError>
    where
        T: PartialOrd + fmt::Display,
    {
        let value = match self.params.get(name) {
            Some(raw) => parse(raw).ok_or_else(|| {
                CommandError::new(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, raw
                ))
            })?,
            None => default.ok_or_else(|| self.missing(name))?,
        };
        if let Some(min) = minval {
            if value < min {
                return Err(self.range_error(name, "minimum", min));
            }
        }
        if let Some(max) = maxval {
            if value > max {
                return Err(self.range_error(name, "maximum", max));
            }
        }
        if let Some(above) = above {
            if value <= above {
                return Err(self.range_error(name, "above", above));
            }
        }
        if let Some(below) = below {
            if value >= below {
                return Err(self.range_error(name, "below", below));
            }
        }
        Ok(value)
    }

    /// A required integer parameter.
    ///
    /// # Errors
    /// Returns [`CommandError`] when it is absent or not an integer.
    pub fn get_int(&self, name: &str) -> Result<i64, CommandError> {
        self.get(name, None, parse_int, None, None, None, None)
    }

    /// An integer parameter, or `default` when it is absent.
    ///
    /// # Errors
    /// Returns [`CommandError`] when it is present but not an integer.
    pub fn get_int_default(&self, name: &str, default: i64) -> Result<i64, CommandError> {
        self.get(name, Some(default), parse_int, None, None, None, None)
    }

    /// A required integer within `minval`/`maxval` (upstream's `get_int`).
    ///
    /// # Errors
    /// As [`GcodeCommand::get_int`], plus a bound failure.
    pub fn get_int_bounded(
        &self,
        name: &str,
        minval: Option<i64>,
        maxval: Option<i64>,
    ) -> Result<i64, CommandError> {
        self.get(name, None, parse_int, minval, maxval, None, None)
    }

    /// A required float parameter.
    ///
    /// # Errors
    /// Returns [`CommandError`] when it is absent or not a number.
    pub fn get_float(&self, name: &str) -> Result<f64, CommandError> {
        self.get(name, None, parse_float, None, None, None, None)
    }

    /// A float parameter, or `default` when it is absent.
    ///
    /// # Errors
    /// Returns [`CommandError`] when it is present but not a number.
    pub fn get_float_default(&self, name: &str, default: f64) -> Result<f64, CommandError> {
        self.get(name, Some(default), parse_float, None, None, None, None)
    }

    /// A required float strictly above/below a bound (upstream's `get_float`).
    ///
    /// # Errors
    /// As [`GcodeCommand::get_float`], plus a bound failure.
    pub fn get_float_bounded(
        &self,
        name: &str,
        above: Option<f64>,
        below: Option<f64>,
    ) -> Result<f64, CommandError> {
        self.get(name, None, parse_float, None, None, above, below)
    }

    /// A required float parameter that must sit within `min..=max`.
    ///
    /// # Errors
    /// As [`GcodeCommand::get_float`], plus a range failure whose message is
    /// upstream's ("must have minimum of …" / "must have maximum of …").
    pub fn get_float_range(&self, name: &str, min: f64, max: f64) -> Result<f64, CommandError> {
        self.get(name, None, parse_float, Some(min), Some(max), None, None)
    }

    /// Send one line to the client, as-is.
    pub fn respond_raw(&self, msg: &str) {
        self.dispatch.respond_raw(msg);
    }

    /// Send an informational line (`// ` prefixed, multi-line joined).
    pub fn respond_info(&self, msg: &str) {
        self.dispatch.respond_info(msg, true);
    }

    /// Send an informational line without logging it.
    ///
    /// Upstream's `respond_info(..., log=False)`: the client sees the line, the
    /// host's log does not. Used by `ECHO` and `HELP`, which report what the
    /// client sent rather than a host event.
    pub fn respond_info_no_log(&self, msg: &str) {
        self.dispatch.respond_info(msg, false);
    }

    /// Acknowledge the line, when its input wants acks.
    ///
    /// Upstream's `ack` (`klippy/gcode.py:54-63`): `ok`, or `ok <msg>`, and only
    /// for a `need_ack` line. Returns whether it acknowledged, which is how
    /// `M115` chooses between `ok <msg>` and an info line.
    pub fn ack(&self, msg: Option<&str>) -> bool {
        if !self.need_ack.swap(false, Ordering::SeqCst) {
            return false;
        }
        match msg {
            Some(msg) => self.respond_raw(&format!("ok {msg}")),
            None => self.respond_raw("ok"),
        }
        true
    }

    fn missing(&self, name: &str) -> CommandError {
        CommandError::new(format!("Error on '{}': missing {}", self.commandline, name))
    }

    fn range_error(&self, name: &str, bound: &str, limit: impl fmt::Display) -> CommandError {
        CommandError::new(format!(
            "Error on '{}': {} must have {} of {}",
            self.commandline, name, bound, limit
        ))
    }
}

// ===========================================================================
// GCodeDispatch
// ===========================================================================

/// A mux command's registrations: one key parameter, a handler per value.
struct Mux {
    key: String,
    values: HashMap<Option<String>, CommandHandler>,
}

struct Commands {
    /// Available once the printer is ready; every registration lands here.
    ready: HashMap<String, CommandHandler>,
    /// Available even before ready (the built-ins a client may need to get out
    /// of trouble).
    base: HashMap<String, CommandHandler>,
    mux: HashMap<String, Mux>,
    help: HashMap<String, String>,
    /// The parameter names each command declared, in declaration order.
    ///
    /// Reported by `get_status` so a client can complete the left of `=`.
    /// A command that declared none has an empty list here, and its status
    /// entry keeps the shape it had before this table existed.
    params: HashMap<String, Vec<String>>,
}

impl Commands {
    fn active(&self, ready: bool) -> &HashMap<String, CommandHandler> {
        if ready {
            &self.ready
        } else {
            &self.base
        }
    }
}

struct Inner {
    /// For the state message, shutdown, and exit requests.
    ///
    /// `Weak`: the dispatcher is a *host* object (`Printer::new` creates it and
    /// `Printer::teardown` deliberately keeps it across restarts), so a strong
    /// handle here would close the loop `printer -> objects -> gcode -> printer`.
    /// Nothing would ever drop the printer — and with it every config object,
    /// including each `Mcu` whose `Drop` releases the receive task's blocking
    /// read. See `a_case_leaves_no_live_mcu_behind`.
    printer: Weak<Printer>,
    ready: AtomicBool,
    commands: Mutex<Commands>,
    /// Where output goes. `register_output_handler` adds; `gcode/subscribe_output`
    /// will be the first client-facing one.
    outputs: Mutex<Vec<Arc<dyn OutputHandler>>>,
}

/// The `gcode` printer object: the command table and the script runner.
pub struct GCodeDispatch {
    inner: Arc<Inner>,
}

impl GCodeDispatch {
    /// Build the dispatcher over `printer` and register the built-in commands.
    ///
    /// The built-ins are available before the printer is ready (`when_not_ready`
    /// upstream): a client that needs `M112` or `STATUS` should not have to wait
    /// for a config file to load.
    pub fn new(printer: Arc<Printer>) -> Self {
        let dispatch = Self {
            inner: Arc::new(Inner {
                printer: Arc::downgrade(&printer),
                ready: AtomicBool::new(false),
                commands: Mutex::new(Commands {
                    ready: HashMap::new(),
                    base: HashMap::new(),
                    mux: HashMap::new(),
                    help: HashMap::new(),
                    params: HashMap::new(),
                }),
                outputs: Mutex::new(Vec::new()),
            }),
        };

        {
            let inner = Arc::clone(&dispatch.inner);
            printer.register_event_handler(
                KlippyEvent::KlippyReady,
                Box::new(move |_| {
                    inner.set_ready(true);
                    inner.respond_info("Klipper state: Ready", false);
                }),
            );
        }
        {
            let inner = Arc::clone(&dispatch.inner);
            printer.register_event_handler(
                KlippyEvent::KlippyShutdown,
                Box::new(move |_| {
                    // Upstream returns early when the printer was already not
                    // ready (`_handle_shutdown`, `klippy/gcode.py:186-193`), so
                    // only the first shutdown prints the state line.
                    if inner.ready.load(Ordering::SeqCst) {
                        inner.set_ready(false);
                        inner.respond_info("Klipper state: Shutdown", false);
                    }
                }),
            );
        }
        {
            let inner = Arc::clone(&dispatch.inner);
            printer.register_event_handler(
                KlippyEvent::KlippyDisconnect,
                Box::new(move |_| inner.respond_info("Klipper state: Disconnect", false)),
            );
        }

        dispatch.register_builtins();
        dispatch
    }

    /// The printer this dispatcher reports to (`gcode:request_restart` and
    /// the shutdown message). `Weak` by design: see `Inner::printer`.
    /// friends are sent there).
    ///
    /// `None` once the printer is gone, which is the end of the process.
    pub fn printer(&self) -> Option<Arc<Printer>> {
        self.inner.printer.upgrade()
    }

    /// Request a restart with the given reason — upstream
    /// `gcode.request_restart` (`klippy/gcode.py:352`). Exposed so a module
    /// that must finish file work (a `SAVE_CONFIG`) can trigger the reload
    /// after it is done.
    pub(crate) fn request_restart(&self, result: &str) {
        self.inner.request_restart(result);
    }

    /// Register a command handler.
    ///
    /// `when_not_ready` keeps it available before the printer is ready, which is
    /// only for the built-ins.
    ///
    /// A module that also knows the command's parameter names registers with
    /// [`GCodeDispatch::register_command_with_params`] so a client can complete
    /// them; this form declares none.
    ///
    /// # Errors
    /// Returns a message when the name is malformed or already registered — a
    /// wiring mistake in klippy, reported at config load like upstream's
    /// `config_error`.
    pub fn register_command(
        &self,
        name: &str,
        handler: CommandHandler,
        desc: Option<&str>,
        when_not_ready: bool,
    ) -> Result<(), String> {
        self.register_command_with_params(name, handler, desc, &[], when_not_ready)
    }

    /// Register a command handler and the parameter names it accepts.
    ///
    /// `params` is the list of `KEY` names the command reads, in the order a
    /// client should offer them. It is **additive**: the names are reported in
    /// `status.gcode.commands[<name>]["parameters"]` for a client that completes
    /// the left of `=`; the command's own behaviour does not depend on the list,
    /// and a command registered without one reports exactly what it did before.
    ///
    /// `when_not_ready` keeps it available before the printer is ready, which is
    /// only for the built-ins.
    ///
    /// # Errors
    /// Returns a message when the name is malformed or already registered — a
    /// wiring mistake in klippy, reported at config load like upstream's
    /// `config_error`.
    pub fn register_command_with_params(
        &self,
        cmd: &str,
        handler: CommandHandler,
        desc: Option<&str>,
        params: &[&str],
        when_not_ready: bool,
    ) -> Result<(), String> {
        if !is_traditional_gcode(cmd) && !is_valid_extended_name(cmd) {
            return Err(format!("Can't register '{cmd}' as it is an invalid name"));
        }
        let mut commands = self.lock();
        if commands.ready.contains_key(cmd) {
            return Err(format!("gcode command {cmd} already registered"));
        }
        commands.ready.insert(cmd.to_string(), Arc::clone(&handler));
        if when_not_ready {
            commands.base.insert(cmd.to_string(), handler);
        }
        if let Some(desc) = desc {
            commands.help.insert(cmd.to_string(), desc.to_string());
        }
        // Always stored, empty list included: a name unregistered and registered
        // again (a homing override) must not keep its previous declaration.
        commands.params.insert(
            cmd.to_string(),
            params.iter().map(|param| (*param).to_string()).collect(),
        );
        Ok(())
    }

    /// Whether a command name is registered (in the ready table, which holds
    /// every registration — `when_not_ready` built-ins land here too). Used by
    /// `gcode_macro`'s load-time check that a macro body's static command
    /// references will resolve at run time.
    pub fn command_exists(&self, name: &str) -> bool {
        self.lock().ready.contains_key(name)
    }

    /// Remove a registered command, as upstream's `register_command(cmd, None)`.
    ///
    /// Returns the handler that was registered, so a module can chain to it (a
    /// `gcode_macro` alias, a homing override). An unknown name returns `None`,
    /// which upstream also treats as a no-op rather than an error.
    ///
    /// The help text is left behind, as upstream's does: the command is gone
    /// from the active table, so `HELP` and `get_status` no longer show it.
    pub fn unregister_command(&self, name: &str) -> Option<CommandHandler> {
        let mut commands = self.lock();
        let old = commands.ready.remove(name);
        commands.base.remove(name);
        old
    }

    /// Register one value of a mux command — a command whose handler depends on
    /// one parameter (`SET_PIN PIN=<name>`).
    ///
    /// The first registration fixes the key parameter; every later one must
    /// agree, and no value may be taken twice.
    ///
    /// A module that also knows the command's other parameter names registers
    /// with [`GCodeDispatch::register_mux_command_with_params`]; this form
    /// declares only the key.
    ///
    /// # Errors
    /// As [`GCodeDispatch::register_command`], plus the mux conflicts upstream
    /// reports.
    pub fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        handler: CommandHandler,
        desc: Option<&str>,
    ) -> Result<(), String> {
        self.register_mux_command_with_params(cmd, key, value, handler, desc, &[])
    }

    /// Register one value of a mux command and the command's parameter names.
    ///
    /// The mux `key` is itself a parameter of the command, so it is **prepended**
    /// to `params` and de-duplicated against it: a caller lists only the other
    /// names, and the key still comes first, in its natural position.
    ///
    /// As [`GCodeDispatch::register_mux_command`], the first value fixes the key
    /// and every later one must agree. The declared lists of one command
    /// **merge** across its values, in first-seen order.
    ///
    /// # Errors
    /// As [`GCodeDispatch::register_mux_command`].
    pub fn register_mux_command_with_params(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        handler: CommandHandler,
        desc: Option<&str>,
        params: &[&str],
    ) -> Result<(), String> {
        // The key leads the declared list, and appears once even when the caller
        // also lists it among the other names.
        let mut declared = Vec::with_capacity(params.len() + 1);
        declared.push(key);
        for param in params {
            if !declared.contains(param) {
                declared.push(param);
            }
        }
        if self.lock().mux.contains_key(cmd) {
            let mut commands = self.lock();
            let mux = commands.mux.get_mut(cmd).expect("checked");
            if mux.key != key {
                return Err(format!(
                    "mux command {cmd} {key} {value:?} may have only one key ({})",
                    mux.key
                ));
            }
            if mux.values.contains_key(&value.map(str::to_string)) {
                return Err(format!(
                    "mux command {cmd} {key} {value:?} already registered"
                ));
            }
            mux.values.insert(value.map(str::to_string), handler);
            // A mux command's values **coexist**, each registered by whatever
            // module owns it: `[output_pin]` and `[pwm_tool]` add a `VALUE`
            // value to `SET_PIN`, `[pwm_cycle_time]` adds one that also takes
            // `CYCLE_TIME`. A client completes the command once, so a name that
            // only a later value declares disappears unless it joins the list
            // here. Names already present stay where they first appeared, and a
            // value that declares nothing leaves the list as it is.
            let declared_names = commands.params.entry(cmd.to_string()).or_default();
            for param in &declared {
                if !declared_names.iter().any(|existing| existing == param) {
                    declared_names.push((*param).to_string());
                }
            }
            if let Some(desc) = desc {
                commands.help.insert(cmd.to_string(), desc.to_string());
            }
            return Ok(());
        }

        // First value: install the dispatcher itself, then the value.
        //
        // The dispatcher is stored in the table it belongs to, so it must hold
        // the dispatcher **weakly**: a strong handle is a self-cycle
        // (`Inner.commands -> handler -> Arc<Inner>`) that keeps the whole
        // dispatcher — and every resource its handlers captured, up to a
        // connected MCU — alive after a restart drops the machine's parts.
        let inner = Arc::downgrade(&self.inner);
        let command = cmd.to_string();
        let dispatcher: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let inner = upgrade(&inner);
            let command = command.clone();
            Box::pin(async move { dispatch_mux(&inner, &command, gcmd).await })
        });
        self.register_command_with_params(cmd, dispatcher, desc, &declared, false)?;
        self.lock().mux.insert(
            cmd.to_string(),
            Mux {
                key: key.to_string(),
                values: HashMap::from([(value.map(str::to_string), handler)]),
            },
        );
        Ok(())
    }

    /// Run a script from inside a command handler.
    ///
    /// Upstream's `run_script_from_command` (`klippy/gcode.py:237-238`): the
    /// entry point a handler uses so a module such as `gcode_macro` can run
    /// another script. Upstream holds the dispatcher's mutex only in
    /// `run_script`; this host serialises scripts on the calling task, so both
    /// run the same way — the name is kept because the consumers use it.
    ///
    /// # Errors
    /// Returns the first [`CommandError`] the script produced.
    pub async fn run_script_from_command(&self, script: &str) -> Result<(), CommandError> {
        for line in script.split('\n') {
            // An API `gcode/script` line is not acknowledged; the file/serial
            // input protocol is the only `need_ack` producer (`gcode.py:210`).
            process_line(&self.inner, line, false).await?;
        }
        Ok(())
    }

    /// Run a script: split on newlines, run each line, stop at the first error.
    ///
    /// The error is reported to the output (as `!! …`) by `process_line`, as
    /// upstream does, so a client subscribed to output sees why the script
    /// stopped even when it ignores the reply.
    ///
    /// # Errors
    /// Returns the first [`CommandError`] the script produced.
    pub async fn run_script(&self, script: &str) -> Result<(), CommandError> {
        self.run_script_from_command(script).await
    }

    /// Run a script from a synchronous context (tests and simple callers).
    ///
    /// A thin `block_on` over [`GCodeDispatch::run_script`]: production code
    /// awaits the async form; a test that has no runtime gets one here rather
    /// than duplicating the `#[tokio::test]` attribute on every case.
    #[cfg(test)]
    pub fn run_script_sync(&self, script: &str) -> Result<(), CommandError> {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("a current-thread runtime")
            .block_on(self.run_script(script))
    }

    /// [`GCodeDispatch::run_script_from_command`] from a synchronous context.
    #[cfg(test)]
    pub fn run_script_from_command_sync(&self, script: &str) -> Result<(), CommandError> {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("a current-thread runtime")
            .block_on(self.run_script_from_command(script))
    }

    /// Build a command for a handler to run, without parsing a line.
    ///
    /// Upstream's `create_gcode_command` (`klippy/gcode.py:244-245`): used by
    /// modules that synthesise a command and hand it to another handler
    /// (`homing`, `probe`, `safe_z_home`, `bed_mesh`, `gcode_arcs`). The line is
    /// never acknowledged, as upstream's is not.
    pub fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: HashMap<String, String>,
    ) -> GcodeCommand {
        GcodeCommand {
            dispatch: Arc::clone(&self.inner),
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            need_ack: AtomicBool::new(false),
        }
    }

    /// Add an output handler, called for every line the dispatcher emits.
    ///
    /// A handler that reports [`OutputHandler::is_closed`] is dropped the next
    /// time a line is emitted, which is how a disconnected subscriber stops
    /// being called.
    pub fn register_output_handler(&self, handler: Arc<dyn OutputHandler>) {
        self.inner
            .outputs
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(handler);
    }

    /// The command table with help text, as `gcode/help` and the status report
    /// it. Upstream's `get_command_help`.
    pub fn command_help(&self) -> HashMap<String, String> {
        self.lock().help.clone()
    }

    fn register_builtins(&self) {
        let simple: [(&str, Option<&str>); 2] = [("M110", None), ("M115", None)];
        for (name, desc) in simple {
            let handler: CommandHandler = match name {
                // Set Current Line Number: accepted and ignored.
                "M110" => sync(|_| Ok(())),
                // Get Firmware Version and Capabilities.
                "M115" => {
                    let printer = self.inner.printer.clone();
                    sync(move |gcmd: &GcodeCommand| {
                        // The host's own version, from the start arguments
                        // (`start_args['software_version']`); the crate version
                        // is the fallback before the host sets them.
                        let version = printer
                            .upgrade()
                            .map(|printer| printer.software_version())
                            .unwrap_or_else(|| env!("CARGO_PKG_VERSION").to_string());
                        let msg = format!("FIRMWARE_NAME:Klipper FIRMWARE_VERSION:{version}");
                        // A file-input line gets `ok <msg>`; an API line gets the
                        // info line instead (`klippy/gcode.py:344-350`).
                        if !gcmd.ack(Some(&msg)) {
                            gcmd.respond_info(&msg);
                        }
                        Ok(())
                    })
                }
                _ => unreachable!(),
            };
            self.register_command(name, handler, desc, true)
                .expect("the built-in command names are valid and unique");
        }

        // The built-in handlers are stored in the table they call into, so each
        // one holds the dispatcher **weakly** (`register_mux_command` explains
        // the cycle). `upgrade` succeeds because a handler only runs while the
        // dispatcher that owns it is alive.
        {
            let inner = Arc::downgrade(&self.inner);
            self.register_command(
                "M112",
                sync(move |_| {
                    if let Some(printer) = upgrade(&inner).printer.upgrade() {
                        printer.invoke_shutdown("Shutdown due to M112 command");
                    }
                    Ok(())
                }),
                None,
                true,
            )
            .expect("M112 is a valid, unique command name");
        }
        for (name, result, desc) in [
            (
                "RESTART",
                "restart",
                "Reload config file and restart host software",
            ),
            (
                "FIRMWARE_RESTART",
                "firmware_restart",
                "Restart firmware, host, and reload config",
            ),
        ] {
            let inner = Arc::downgrade(&self.inner);
            self.register_command(
                name,
                sync(move |_| {
                    upgrade(&inner).request_restart(result);
                    Ok(())
                }),
                Some(desc),
                true,
            )
            .expect("the restart command names are valid and unique");
        }
        {
            self.register_command(
                "ECHO",
                sync(|gcmd: &GcodeCommand| {
                    gcmd.respond_info_no_log(gcmd.commandline());
                    Ok(())
                }),
                None,
                true,
            )
            .expect("ECHO is a valid, unique command name");
        }
        {
            let inner = Arc::downgrade(&self.inner);
            self.register_command(
                "STATUS",
                sync(move |gcmd: &GcodeCommand| cmd_status(&upgrade(&inner), gcmd)),
                Some("Report the printer status"),
                true,
            )
            .expect("STATUS is a valid, unique command name");
        }
        {
            let inner = Arc::downgrade(&self.inner);
            self.register_command(
                "HELP",
                sync(move |gcmd: &GcodeCommand| cmd_help(&upgrade(&inner), gcmd)),
                Some("Report the list of available extended G-Code commands"),
                true,
            )
            .expect("HELP is a valid, unique command name");
        }
    }

    /// Send an informational line from a part that holds no command
    /// (the manual-probe helper's callbacks, a calibration finalizer):
    /// upstream's `gcode.respond_info(msg, log=True)`.
    pub fn respond_info(&self, msg: &str, log: bool) {
        self.inner.respond_info(msg, log);
    }

    /// Send a raw line to all output handlers — upstream's
    /// `gcode.respond_raw(msg)` (`klippy/gcode.py:247-249`). Used by
    /// `virtual_sdcard`'s replay loop, which has no `GcodeCommand` in hand.
    pub fn respond_raw(&self, msg: &str) {
        self.inner.respond_raw(msg);
    }

    fn lock(&self) -> MutexGuard<'_, Commands> {
        self.inner
            .commands
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for GCodeDispatch {
    /// The command table, as upstream's `GCodeDispatch.get_status`.
    ///
    /// Built from the **active** table, so before the printer is ready only the
    /// base commands show up — upstream caches the same thing in
    /// `_build_status_commands` (`klippy/gcode.py:176-184`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let ready = self.inner.ready.load(Ordering::SeqCst);
        let commands = self.lock();
        let mut status = Map::new();
        for name in commands.active(ready).keys() {
            let mut entry = Map::new();
            if let Some(help) = commands.help.get(name) {
                entry.insert("help".to_string(), Value::String(help.clone()));
            }
            // Only a command that declared names gets the field: one that
            // declared none reports the plain object it did before.
            if let Some(params) = commands
                .params
                .get(name)
                .filter(|params| !params.is_empty())
            {
                entry.insert(
                    "parameters".to_string(),
                    Value::Array(
                        params
                            .iter()
                            .map(|param| Value::String(param.clone()))
                            .collect(),
                    ),
                );
            }
            status.insert(name.clone(), Value::Object(entry));
        }
        json!({ "commands": status })
    }
}

// ===========================================================================
// Dispatch
// ===========================================================================

/// Upgrade a stored handler's weak handle to the dispatcher it belongs to.
///
/// The command handlers in `Inner.commands` hold the dispatcher **weakly**: a
/// strong handle is a self-cycle (`Inner.commands -> handler -> Arc<Inner>`) that
/// would keep the dispatcher — and every resource its handlers captured, up to a
/// connected MCU — alive after a restart drops the machine's parts
/// (`Printer::teardown`). A handler only runs while the dispatcher that owns it
/// is alive, so the upgrade always succeeds.
fn upgrade(inner: &Weak<Inner>) -> Arc<Inner> {
    inner
        .upgrade()
        .expect("the dispatcher outlives the command handlers it stores")
}

/// One line of a script: parse, find the handler, run it.
async fn process_line(inner: &Arc<Inner>, line: &str, need_ack: bool) -> Result<(), CommandError> {
    let Some(parsed) = parse_line(line) else {
        return Ok(()); // blank or comment-only
    };
    let ready = inner.ready.load(Ordering::SeqCst);

    let handler = {
        let commands = inner
            .commands
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        commands.active(ready).get(&parsed.command).cloned()
    };

    // An unregistered command is never re-parsed: upstream hands the split
    // parameters to `cmd_default`. A registered extended command has them
    // re-parsed inside the handler (`_get_extended_params`), so a malformed
    // line is an ordinary command error.
    let mut gcmd = GcodeCommand {
        dispatch: Arc::clone(inner),
        command: parsed.command.clone(),
        commandline: parsed.commandline.clone(),
        params: parsed.params.clone(),
        need_ack: AtomicBool::new(need_ack),
    };

    let outcome = {
        let fut: CommandFuture<'_> = match &handler {
            Some(handler) => {
                if parsed.traditional {
                    handler(&gcmd)
                } else {
                    match parse_extended(&parsed.raw_params, &parsed.commandline) {
                        Ok(params) => {
                            gcmd.params = params;
                            handler(&gcmd)
                        }
                        Err(err) => Box::pin(std::future::ready(Err(err))),
                    }
                }
            }
            None => Box::pin(async { default_handler(inner, &mut gcmd).await }),
        };
        match catch_unwind_future(fut).await {
            Ok(Ok(())) => HandlerOutcome::Ok,
            Ok(Err(err)) => HandlerOutcome::CommandError(err),
            Err(_) => {
                let msg = format!("Internal error on command:\"{}\"", parsed.command);
                error!("{msg}");
                if let Some(printer) = inner.printer.upgrade() {
                    printer.invoke_shutdown(&msg);
                }
                HandlerOutcome::Internal(msg)
            }
        }
    };

    match outcome {
        HandlerOutcome::Ok => {
            gcmd.ack(None);
            Ok(())
        }
        HandlerOutcome::CommandError(err) => {
            // A command error is the user's problem: report it, tell the parts
            // that listen, and only stop the script when the line was not
            // acknowledged (`klippy/gcode.py:223-228`).
            inner.respond_error(err.message());
            if let Some(printer) = inner.printer.upgrade() {
                printer.send_event(&KlippyEvent::GcodeCommandError);
            }
            if need_ack {
                gcmd.ack(None);
                Ok(())
            } else {
                Err(err)
            }
        }
        HandlerOutcome::Internal(msg) => {
            // The printer was already shut down above; the client is still told.
            // No `gcode:command_error`: upstream fires it only for a
            // `CommandError` (`klippy/gcode.py:224-226`).
            inner.respond_error(&msg);
            if need_ack {
                gcmd.ack(None);
                Ok(())
            } else {
                Err(CommandError::new(msg))
            }
        }
    }
}

/// Run a boxed handler future, turning a panic into an `Err` payload.
///
/// `catch_unwind` cannot cross an `.await`, so the panic is caught around each
/// **poll** of the future. This is what replaces the synchronous
/// `catch_unwind` the dispatcher used before handlers could await.
async fn catch_unwind_future<'a>(
    mut fut: CommandFuture<'a>,
) -> Result<Result<(), CommandError>, Box<dyn std::any::Any + Send>> {
    std::future::poll_fn(move |cx| {
        match std::panic::catch_unwind(AssertUnwindSafe(|| fut.as_mut().poll(cx))) {
            Ok(poll) => poll.map(Ok),
            Err(payload) => std::task::Poll::Ready(Err(payload)),
        }
    })
    .await
}

/// What running one command handler produced.
enum HandlerOutcome {
    /// The handler succeeded.
    Ok,
    /// The handler reported a user error.
    CommandError(CommandError),
    /// The handler panicked; the printer was shut down.
    Internal(String),
}

/// The handler for an unregistered command (`klippy/gcode.py:283-316`).
///
/// Most of this is upstream's list of requests a slicer sends for a module this
/// host may not have: they are answered quietly instead of as unknown commands.
async fn default_handler(inner: &Arc<Inner>, gcmd: &mut GcodeCommand) -> Result<(), CommandError> {
    let command = gcmd.command.clone();

    // Temperature and SD-card requests are answered before the ready check, so
    // a client polling them during startup is not told the printer is not ready.
    if command == "M105" {
        gcmd.ack(Some("T:0"));
        return Ok(());
    }
    if command == "M21" {
        return Ok(());
    }
    if !inner.ready.load(Ordering::SeqCst) {
        return Err(CommandError::new(
            inner
                .printer
                .upgrade()
                .map(|printer| printer.get_state_message().message)
                .unwrap_or_else(|| "Klipper state: Disconnect".to_string()),
        ));
    }
    if command.is_empty() {
        return Ok(());
    }

    if let Some((real, _)) = command.split_once(' ') {
        // `M117 <message>`: a display message whose text is not a parameter.
        // If the module registered the command, hand it the line unchanged.
        if matches!(real, "M117" | "M118" | "M23") {
            let handler = {
                let commands = inner
                    .commands
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                commands.active(true).get(real).cloned()
            };
            if let Some(handler) = handler {
                gcmd.command = real.to_string();
                return handler(gcmd).await;
            }
        }
    } else if matches!(command.as_str(), "M140" | "M104")
        && gcmd.get_float_default("S", 0.0)? == 0.0
    {
        // A request to turn off a heater that is not present.
        return Ok(());
    } else if command == "M107" || (command == "M106" && gcmd.get_float_default("S", 1.0)? == 0.0) {
        // A request to turn off a fan that is not present.
        return Ok(());
    }

    inner.respond_info(&format!("Unknown command:\"{}\"", command), false);
    Ok(())
}

/// `STATUS`: report the printer state, as an error when it is not ready
/// (`klippy/gcode.py:371-378`).
fn cmd_status(inner: &Arc<Inner>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    if inner.ready.load(Ordering::SeqCst) {
        gcmd.respond_info("Klipper state: Ready");
        return Ok(());
    }
    let msg = inner
        .printer
        .upgrade()
        .map(|printer| printer.get_state_message().message)
        .unwrap_or_else(|| "Klipper state: Disconnect".to_string());
    Err(CommandError::new(format!(
        "{}\nKlipper state: Not ready",
        msg.trim_end()
    )))
}

/// `HELP`: list the extended commands that carry help text
/// (`klippy/gcode.py:379-389`).
///
/// Lists the **active** table, so before the printer is ready it names the base
/// commands and says so; the line is not logged (`log=False`).
fn cmd_help(inner: &Arc<Inner>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let ready = inner.ready.load(Ordering::SeqCst);
    let commands = inner
        .commands
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let mut lines = Vec::new();
    if !ready {
        lines.push("Printer is not ready - not all commands available.".to_string());
    }
    lines.push("Available extended commands:".to_string());
    let mut names: Vec<&String> = commands
        .active(ready)
        .keys()
        .filter(|name| commands.help.contains_key(*name))
        .collect();
    names.sort();
    for name in names {
        lines.push(format!("{:<10}: {}", name, commands.help[name]));
    }
    gcmd.respond_info_no_log(&lines.join("\n"));
    Ok(())
}

/// Dispatch a mux command to the handler registered for its key value
/// (`klippy/gcode.py:317-337`).
async fn dispatch_mux(
    inner: &Arc<Inner>,
    cmd: &str,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let (key, has_default) = {
        let commands = inner
            .commands
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mux = commands
            .mux
            .get(cmd)
            .expect("a mux dispatcher is only installed with its values");
        (mux.key.clone(), mux.values.contains_key(&None))
    };

    // With a default registered, a request that omits the key selects it (the
    // `None` entry); without one the key is required. Looking the request up as
    // `Some("")` would never match the default and would report a bogus value
    // (`_cmd_mux`, `klippy/gcode.py:317-342`).
    let requested: Option<String> = if has_default {
        gcmd.get_command_parameters().get(&key).cloned()
    } else {
        Some(gcmd.get_str(&key)?)
    };

    let handler = {
        let commands = inner
            .commands
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mux = commands.mux.get(cmd).expect("checked above");
        mux.values.get(&requested).map(Arc::clone)
    };
    if let Some(handler) = handler {
        return handler(gcmd).await;
    }

    let requested = requested.unwrap_or_default();

    // Not a registered value: report what is available (upstream's wording).
    let mut values: Vec<String> = {
        let commands = inner
            .commands
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        commands.mux[cmd]
            .values
            .keys()
            .filter_map(|value| value.as_ref().map(|v| format!("'{v}'")))
            .collect()
    };
    // A `HashMap` has no order, but the message has to be stable.
    values.sort();
    let guess = values
        .iter()
        .find(|value| !requested.is_empty() && value.contains(&requested))
        .map(|value| format!(". Did you mean {value}?"))
        .unwrap_or_else(|| format!(". Options: {}", values.join(", ")));
    Err(CommandError::new(format!(
        "The value '{requested}' is not valid for {key}{guess}"
    )))
}

impl Inner {
    fn set_ready(&self, ready: bool) {
        self.ready.store(ready, Ordering::SeqCst);
    }

    /// `GCodeDispatch.request_restart` (`klippy/gcode.py:352-362`): with the
    /// printer ready, note the last print time, fire `gcode:request_restart`,
    /// dwell, and wait for the queued moves; then ask the printer to exit.
    pub(crate) fn request_restart(&self, result: &str) {
        // No printer left means nothing to restart: the process is already on
        // its way out (the `Weak` is why this cannot keep it alive).
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if self.ready.load(Ordering::SeqCst) {
            if let Some(hooks) = printer.restart_hooks() {
                let print_time = hooks.get_last_move_time();
                if result == "exit" {
                    info!("Exiting (print time {print_time:.3}s)");
                }
                printer.send_event(&KlippyEvent::GcodeRequestRestart { print_time });
                hooks.dwell(0.500);
                hooks.wait_moves();
            }
        }
        printer.request_exit(result);
    }

    fn respond_raw(&self, msg: &str) {
        let handlers = {
            let mut outputs = self
                .outputs
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            outputs.retain(|handler| !handler.is_closed());
            outputs.clone()
        };
        for handler in handlers {
            handler.emit(msg);
        }
    }

    /// `// ` prefixed lines, optionally logged (`klippy/gcode.py:250-254`).
    fn respond_info(&self, msg: &str, log: bool) {
        if log {
            info!("{}", msg);
        }
        let lines: Vec<&str> = msg.trim().split('\n').map(str::trim).collect();
        self.respond_raw(&format!("// {}", lines.join("\n// ")));
    }

    /// `!! ` for the first line, the rest as info (`klippy/gcode.py:255-262`).
    fn respond_error(&self, msg: &str) {
        warn!("{}", msg);
        let lines: Vec<&str> = msg.trim().split('\n').collect();
        if lines.len() > 1 {
            self.respond_info(&lines.join("\n"), false);
        }
        let first = lines.first().copied().unwrap_or("").trim();
        self.respond_raw(&format!("!! {first}"));
    }
}

// ===========================================================================
// Parsing
// ===========================================================================

/// One parsed line.
struct Parsed {
    command: String,
    /// The line as sent, trimmed, comment included.
    commandline: String,
    /// For a traditional command, the parameters the split produced. For an
    /// extended one they are re-parsed from `raw_params`.
    params: HashMap<String, String>,
    /// The text after the command, for extended re-parsing.
    raw_params: String,
    traditional: bool,
}

/// Whether `cmd` is a traditional command: a letter followed by a digit
/// (`klippy/gcode.py:125-131`).
pub fn is_traditional_gcode(cmd: &str) -> bool {
    let name = cmd.split_whitespace().next().unwrap_or("");
    let mut chars = name.chars();
    let Some(letter) = chars.next() else {
        return false;
    };
    let Some(digit) = chars.next() else {
        return false;
    };
    if !letter.is_ascii_uppercase() || !digit.is_ascii_digit() {
        return false;
    }
    // The rest must be a number, not merely start with one. Upstream tries
    // `float(cmd[1:])` (`klippy/gcode.py:129`), so `M110` is traditional but
    // `I2C_READ` is not: it is an extended name that happens to begin with a
    // letter and a digit, and one upstream rejects at registration.
    name[1..].parse::<f64>().is_ok()
}

/// Upstream's validity rule for an extended command name
/// (`klippy/gcode.py:137-148`): uppercase, no spaces, letters/digits/underscore,
/// not starting with a digit, and not a letter followed by a digit (that would
/// be traditional).
fn is_valid_extended_name(name: &str) -> bool {
    if name.is_empty() || name.to_uppercase() != name {
        return false;
    }
    if !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return false;
    }
    let mut chars = name.chars();
    let first = chars.next().unwrap();
    if first.is_ascii_digit() {
        return false;
    }
    if let Some(second) = chars.next() {
        if second.is_ascii_digit() {
            return false;
        }
    }
    true
}

/// Split a line the way upstream's `args_r` does: the pieces are the text
/// around each run of `[A-Z_]` (or a lone `*`), with the runs kept, i.e. the
/// result of `re.split(r'([A-Z_]+|[A-Z*])', line.upper())`.
fn split_command_parts(upper: &str) -> Vec<&str> {
    let bytes = upper.as_bytes();
    let mut parts = Vec::new();
    let mut last = 0;
    let mut i = 0;
    while i < bytes.len() {
        let byte = bytes[i];
        if byte == b'*' {
            parts.push(&upper[last..i]);
            parts.push("*");
            i += 1;
            last = i;
        } else if byte.is_ascii_uppercase() || byte == b'_' {
            let start = i;
            while i < bytes.len() && (bytes[i].is_ascii_uppercase() || bytes[i] == b'_') {
                i += 1;
            }
            parts.push(&upper[last..start]);
            parts.push(&upper[start..i]);
            last = i;
        } else {
            i += 1;
        }
    }
    parts.push(&upper[last..]);
    parts
}

/// The command name a G-Code line would dispatch to, or `None` for a
/// blank/comment-only line. Wraps [`parse_line`] for callers (the
/// `gcode_macro` load-time checker) that need only the name.
pub fn command_of_line(line: &str) -> Option<String> {
    parse_line(line).map(|parsed| parsed.command)
}

/// Parse one line, or `None` for a blank/comment-only line.
fn parse_line(line: &str) -> Option<Parsed> {
    let stripped = line.trim();
    let without_comment = match stripped.find(';') {
        Some(pos) => &stripped[..pos],
        None => stripped,
    };
    let upper = without_comment.to_uppercase();
    let parts = split_command_parts(&upper);

    // A leading `N<digits>` is a line number: the command starts at parts[3].
    let has_line_number =
        matches!((parts.first(), parts.get(1)), (Some(a), Some(b)) if format!("{a}{b}") == "N");
    let command = if has_line_number {
        format!(
            "{}{}",
            parts.get(3).copied().unwrap_or(""),
            parts.get(4).copied().unwrap_or("")
        )
    } else {
        format!(
            "{}{}{}",
            parts.first().copied().unwrap_or(""),
            parts.get(1).copied().unwrap_or(""),
            parts.get(2).copied().unwrap_or("")
        )
    };
    let command = command.trim().to_string();
    if command.is_empty() {
        return None;
    }

    let mut params = HashMap::new();
    let mut index = 1;
    while index + 1 < parts.len() {
        params.insert(
            parts[index].to_string(),
            parts[index + 1].trim().to_string(),
        );
        index += 2;
    }

    let raw_params = raw_parameters(without_comment);
    let traditional = is_traditional_gcode(&command);
    Some(Parsed {
        command,
        commandline: stripped.to_string(),
        params,
        raw_params,
        traditional,
    })
}

/// The text after the command, for extended re-parsing.
///
/// The command is the token after an optional line number, so the remainder is
/// simply what follows it — plus, on a line-numbered line, upstream's trailing
/// `*<digits>` checksum handling (`get_raw_command_parameters`,
/// `klippy/gcode.py:40-51`; without a line number upstream leaves the text
/// alone).
fn raw_parameters(without_comment: &str) -> String {
    let mut tokens = without_comment.split_whitespace();
    let first = tokens.next().unwrap_or("");
    let line_number = first.len() > 1
        && first[..1].eq_ignore_ascii_case("N")
        && first[1..].chars().all(|c| c.is_ascii_digit());
    if line_number {
        let _ = tokens.next();
    }
    let rest = tokens.collect::<Vec<_>>().join(" ");
    if !line_number {
        return rest;
    }
    match rest.rfind('*') {
        Some(pos)
            if !rest[pos + 1..].is_empty()
                && rest[pos + 1..].chars().all(|c| c.is_ascii_digit()) =>
        {
            rest[..pos].to_string()
        }
        _ => rest,
    }
}

/// Parse extended parameters: whitespace-separated `KEY=VALUE`, with shell
/// quoting, backslash escapes, and `#`/`;` comments (`klippy/gcode.py:266-281`,
/// whose `shlex` does the quoting).
///
/// A backslash is dropped outside single quotes; inside double quotes it only
/// escapes `"` and `\`, so `"a\db"` stays `a\db`. Adjacent quoted and unquoted
/// pieces join into one token (`a"b"c` is `abc`).
///
/// # Errors
/// Returns [`CommandError`] when a token has no `=`, a quote is unterminated, or
/// the text ends with a dangling backslash — upstream's "Malformed command".
fn parse_extended(raw: &str, commandline: &str) -> Result<HashMap<String, String>, CommandError> {
    let malformed = || CommandError::new(format!("Malformed command '{commandline}'"));

    let mut tokens = Vec::new();
    let mut token = String::new();
    let mut in_single = false;
    let mut in_double = false;
    let mut escaped = false;
    for c in raw.chars() {
        if escaped {
            if in_double && c != '"' && c != '\\' {
                token.push('\\');
            }
            token.push(c);
            escaped = false;
        } else if in_single {
            if c == '\'' {
                in_single = false;
            } else {
                token.push(c);
            }
        } else if in_double {
            match c {
                '"' => in_double = false,
                '\\' => escaped = true,
                c => token.push(c),
            }
        } else {
            match c {
                '\'' => in_single = true,
                '"' => in_double = true,
                '\\' => escaped = true,
                '#' | ';' => break,
                c if c.is_whitespace() => {
                    if !token.is_empty() {
                        tokens.push(std::mem::take(&mut token));
                    }
                }
                c => token.push(c),
            }
        }
    }
    if in_single || in_double || escaped {
        return Err(malformed());
    }
    if !token.is_empty() {
        tokens.push(token);
    }

    let mut params = HashMap::new();
    for token in tokens {
        let (key, value) = token.split_once('=').ok_or_else(&malformed)?;
        params.insert(key.to_uppercase(), value.to_string());
    }
    Ok(params)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::atomic::AtomicUsize;

    /// A dispatcher over a fresh printer, with an output collector.
    /// The dispatcher plus the printer it reports to.
    ///
    /// The printer is returned (and must be kept) because the dispatcher holds
    /// it **weakly**: a strong handle there would close the
    /// `printer -> objects -> gcode -> printer` cycle and leak the whole machine
    /// (see `GCodeDispatch`'s `Inner::printer`).
    fn dispatch_with_printer() -> (Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>, Arc<Printer>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            dispatch.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }
        (dispatch, output, printer)
    }

    /// The dispatcher alone, for tests that do not need the printer.
    fn dispatch() -> (Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>) {
        let (dispatch, output, _printer) = dispatch_with_printer();
        (dispatch, output)
    }

    /// The dispatcher with the printer kept alive for the test's duration.
    fn dispatch_keeping_printer() -> (Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>, Arc<Printer>) {
        dispatch_with_printer()
    }

    /// Run a script, ignoring the result.
    fn run(dispatch: &GCodeDispatch, script: &str) {
        let _ = dispatch.run_script_sync(script);
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        output.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// Dropping the dispatcher must actually free it.
    ///
    /// Its built-in handlers (and the mux dispatcher a resource registers, like
    /// `SET_PIN`) live in its own command table. Holding them strongly would make
    /// the dispatcher keep *itself* alive forever — and with it every resource
    /// those handlers captured, up to an MCU connection whose receive task reads
    /// the serial port. That is what turned a `firmware_restart` into a frame
    /// desync: the old connection was never torn down (`Printer::teardown`).
    #[test]
    fn test_dropping_the_dispatcher_frees_its_handlers() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch
            .register_mux_command("SET_PIN", "PIN", Some("led"), sync(|_| Ok(())), None)
            .unwrap();

        let inner = Arc::downgrade(&dispatch.inner);
        drop(dispatch);
        // The dispatcher also registered printer event handlers that hold it;
        // a restart clears those (`Printer::teardown`), leaving only the command
        // table to account for.
        printer.teardown();

        assert!(
            inner.upgrade().is_none(),
            "the command table must not hold the dispatcher alive"
        );
    }

    /// A handler that records the command lines it saw.
    fn recorder() -> (CommandHandler, Arc<Mutex<Vec<String>>>) {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let handler: CommandHandler = {
            let seen = Arc::clone(&seen);
            sync(move |gcmd: &GcodeCommand| {
                seen.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(gcmd.commandline().to_string());
                Ok(())
            })
        };
        (handler, seen)
    }

    // -----------------------------------------------------------------------
    // Parsing
    // -----------------------------------------------------------------------

    #[test]
    fn test_traditional_commands_split_into_letters_and_values() {
        let parsed = parse_line("G1 X10.5 Y20").unwrap();

        assert_eq!(parsed.command, "G1");
        assert!(parsed.traditional);
        assert_eq!(parsed.params.get("X").map(String::as_str), Some("10.5"));
        assert_eq!(parsed.params.get("Y").map(String::as_str), Some("20"));
    }

    #[test]
    fn test_a_line_number_is_skipped() {
        let parsed = parse_line("N5 M110").unwrap();

        assert_eq!(parsed.command, "M110");
        assert_eq!(parsed.raw_params, "");
    }

    #[test]
    fn test_a_line_number_drops_a_trailing_checksum() {
        // A line-numbered line may end in `*<digits>`; upstream strips it from
        // the raw parameters (`get_raw_command_parameters`).
        let parsed = parse_line("N5 SET_PIN PIN=fan VALUE=1*45").unwrap();
        assert_eq!(parsed.raw_params, "PIN=fan VALUE=1");

        // Without a line number the text is left as written.
        let parsed = parse_line("SET_PIN PIN=fan VALUE=1*45").unwrap();
        assert_eq!(parsed.raw_params, "PIN=fan VALUE=1*45");
    }

    #[test]
    fn test_comments_and_blank_lines_are_ignored() {
        assert!(parse_line("").is_none());
        assert!(parse_line("   ").is_none());
        assert!(parse_line("; just a comment").is_none());

        let parsed = parse_line("M110 ; reset").unwrap();
        assert_eq!(parsed.command, "M110");
        assert_eq!(parsed.commandline, "M110 ; reset");
    }

    #[test]
    fn test_extended_parameters_are_key_value_with_quoting() {
        let parsed = parse_line("SET_PIN PIN=fan VALUE=1").unwrap();

        assert_eq!(parsed.command, "SET_PIN");
        assert!(!parsed.traditional);

        let params = parse_extended(&parsed.raw_params, &parsed.commandline).unwrap();
        assert_eq!(params.get("PIN").map(String::as_str), Some("fan"));
        assert_eq!(params.get("VALUE").map(String::as_str), Some("1"));

        // Values keep their case; quoted values keep their spaces.
        let params = parse_extended(r#"PIN=heater MESSAGE="hello world""#, "M").unwrap();
        assert_eq!(
            params.get("MESSAGE").map(String::as_str),
            Some("hello world")
        );
    }

    #[test]
    fn test_a_backslash_escapes_the_next_character() {
        let value = |raw: &str| parse_extended(raw, "X").unwrap()["PIN"].clone();

        // Outside quotes the backslash is dropped.
        assert_eq!(value(r"PIN=a\ b"), "a b");
        assert_eq!(value(r#"PIN=\"q\""#), "\"q\"");
        assert_eq!(value(r"PIN=a\\b"), r"a\b");

        // Inside double quotes only `"` and `\` are escaped; anything else keeps
        // the backslash, as `shlex` does.
        assert_eq!(value(r#"PIN="a\db""#), r"a\db");
        assert_eq!(value(r#"PIN="a\"b""#), "a\"b");
        assert_eq!(value(r#"PIN="a\\b""#), r"a\b");

        // Inside single quotes a backslash is literal.
        assert_eq!(value(r"PIN='a\db'"), r"a\db");

        // Adjacent quoted and unquoted pieces join into one token.
        assert_eq!(value(r#"PIN=a"b"c"#), "abc");

        // A dangling backslash is malformed.
        assert!(parse_extended(r"PIN=a\", "X").is_err());
    }

    #[test]
    fn test_a_malformed_extended_parameter_is_an_error() {
        let err = parse_extended("PINfan", "SET_PIN PINfan").unwrap_err();
        assert_eq!(err.to_string(), "Malformed command 'SET_PIN PINfan'");

        let err = parse_extended(r#"PIN="unterminated"#, "X").unwrap_err();
        assert!(err.to_string().starts_with("Malformed command"), "{err}");
    }

    #[test]
    fn test_name_classification() {
        assert!(is_traditional_gcode("M110"));
        assert!(is_traditional_gcode("G1"));
        assert!(!is_traditional_gcode("SET_PIN"));
        assert!(!is_traditional_gcode("M"));

        assert!(is_valid_extended_name("SET_PIN"));
        assert!(is_valid_extended_name("BED_MESH_CALIBRATE"));
        assert!(!is_valid_extended_name("set_pin"));
        assert!(!is_valid_extended_name("SET PIN"));
        assert!(!is_valid_extended_name("1PIN"));
        // A letter followed by a digit is traditional, not extended.
        assert!(!is_valid_extended_name("M110"));

        // `I2C_READ` begins with a letter and a digit but is not traditional
        // (the rest is not a number), so it is neither form and must be
        // rejected rather than registered as a name nothing can dispatch.
        assert!(!is_traditional_gcode("I2C_READ"));
        assert!(!is_valid_extended_name("I2C_READ"));
        assert!(is_traditional_gcode("M2"));
        assert!(!is_traditional_gcode("M2A"));
    }

    #[test]
    fn test_a_letter_digit_name_that_is_not_traditional_is_refused() {
        let (dispatch, _output) = dispatch();
        let (handler, _) = recorder();

        // The parser would read this line as the command `I2`, so accepting the
        // registration would leave a command that can never run.
        let err = dispatch
            .register_command("I2C_READ", handler, None, false)
            .unwrap_err();

        assert_eq!(err, "Can't register 'I2C_READ' as it is an invalid name");
    }

    // -----------------------------------------------------------------------
    // Registration and dispatch
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_registered_command_runs() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, seen) = recorder();
        dispatch
            .register_command("MY_CMD", handler, None, false)
            .unwrap();

        run(&dispatch, "MY_CMD A=1");

        assert_eq!(*seen.lock().unwrap(), ["MY_CMD A=1"]);
    }

    #[test]
    fn test_registering_the_same_command_twice_is_refused() {
        let (dispatch, _output) = dispatch();
        let (handler, _) = recorder();
        dispatch
            .register_command("MY_CMD", Arc::clone(&handler), None, false)
            .unwrap();

        let err = dispatch
            .register_command("MY_CMD", handler, None, false)
            .unwrap_err();

        assert_eq!(err, "gcode command MY_CMD already registered");
    }

    #[test]
    fn test_a_command_can_be_unregistered() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, seen) = recorder();
        dispatch
            .register_command("MY_CMD", handler, None, false)
            .unwrap();

        let old = dispatch.unregister_command("MY_CMD");

        assert!(old.is_some());
        // Gone from the table: the line is unknown now.
        run(&dispatch, "MY_CMD");
        assert!(seen.lock().unwrap().is_empty());
        assert_eq!(
            emitted(&output).last().map(String::as_str),
            Some("// Unknown command:\"MY_CMD\"")
        );
        // Removing it again is a no-op.
        assert!(dispatch.unregister_command("MY_CMD").is_none());
    }

    #[test]
    fn test_an_invalid_extended_name_is_refused() {
        let (dispatch, _output) = dispatch();
        let (handler, _) = recorder();

        let err = dispatch
            .register_command("bad name", handler, None, false)
            .unwrap_err();

        assert_eq!(err, "Can't register 'bad name' as it is an invalid name");
    }

    #[test]
    fn test_an_unknown_command_is_reported_but_not_an_error() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);

        assert!(dispatch.run_script_sync("NOPE").is_ok());

        assert_eq!(emitted(&output), ["// Unknown command:\"NOPE\""]);
    }

    #[test]
    fn test_unknown_requests_for_missing_modules_are_quiet() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);

        // A slicer asks for modules this host may not have; upstream answers
        // these quietly rather than as unknown commands.
        for line in ["M105", "M21", "M140 S0", "M104 S0", "M107", "M106 S0"] {
            assert!(dispatch.run_script_sync(line).is_ok(), "{line}");
        }
        assert_eq!(emitted(&output), Vec::<String>::new());

        // A request that actually wants heat is still reported.
        run(&dispatch, "M104 S200");
        assert!(emitted(&output)
            .last()
            .map(|l| l.contains("Unknown command"))
            .unwrap_or(false));
    }

    #[test]
    fn test_a_display_message_routes_to_its_registered_command() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);
        let seen = Arc::new(Mutex::new(Vec::new()));
        {
            let seen = Arc::clone(&seen);
            dispatch
                .register_command(
                    "M117",
                    sync(move |gcmd: &GcodeCommand| {
                        seen.lock()
                            .unwrap_or_else(|p| p.into_inner())
                            .push(gcmd.commandline().to_string());
                        Ok(())
                    }),
                    None,
                    false,
                )
                .unwrap();
        }

        // `M117 123` is one command name (`M117 123`); the message is not a
        // parameter, so the line is routed to the registered `M117`.
        run(&dispatch, "M117 123");

        assert_eq!(*seen.lock().unwrap(), ["M117 123"]);
        assert_eq!(emitted(&output), Vec::<String>::new());
    }

    #[test]
    fn test_a_command_error_stops_the_script_and_is_reported() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);
        let calls = Arc::new(AtomicUsize::new(0));
        {
            let calls = Arc::clone(&calls);
            dispatch
                .register_command(
                    "FAIL",
                    sync(move |_| {
                        calls.fetch_add(1, Ordering::SeqCst);
                        Err(CommandError::new("boom"))
                    }),
                    None,
                    false,
                )
                .unwrap();
        }

        let err = dispatch.run_script_sync("FAIL\nNOPE").unwrap_err();

        assert_eq!(err.to_string(), "boom");
        assert_eq!(calls.load(Ordering::SeqCst), 1, "the script must stop");
        let lines = emitted(&output);
        assert_eq!(lines.last().map(String::as_str), Some("!! boom"));
    }

    #[test]
    fn test_a_command_before_ready_reports_the_state() {
        let (dispatch, _output, _printer) = dispatch_keeping_printer();
        let (handler, seen) = recorder();
        dispatch
            .register_command("MY_CMD", handler, None, false)
            .unwrap();

        // Not ready: a ready-only command is not in the active table, and the
        // default reports the state message instead of running it.
        let err = dispatch.run_script_sync("MY_CMD").unwrap_err();

        assert!(err.to_string().contains("Starting up"), "{err}");
        assert!(seen.lock().unwrap().is_empty());
    }

    #[test]
    fn test_a_builtin_is_available_before_ready() {
        let (dispatch, output) = dispatch();

        // M115 is registered when_not_ready, so it runs even in startup.
        assert!(dispatch.run_script_sync("M115").is_ok());

        let lines = emitted(&output);
        assert!(
            lines.iter().any(|l| l.contains("FIRMWARE_NAME:Klipper")),
            "{lines:?}"
        );
    }

    #[test]
    fn test_m115_reports_the_host_software_version() {
        // The version comes from the start arguments (upstream's
        // `start_args['software_version']`), so a host that sets them gets its
        // own version in the reply.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut args = crate::core::klippy::api::StartArgs::collect("/tmp/printer.cfg", None);
        args.software_version = "v9.9.9-test".to_string();
        printer.set_start_args(Arc::new(args));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            dispatch.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }

        dispatch.run_script_sync("M115").unwrap();

        let lines = emitted(&output);
        assert!(
            lines
                .iter()
                .any(|l| l.contains("FIRMWARE_VERSION:v9.9.9-test")),
            "{lines:?}"
        );
    }

    #[test]
    fn test_m112_invokes_shutdown() {
        let (dispatch, _output, _printer) = dispatch_keeping_printer();

        run(&dispatch, "M112");

        assert_eq!(
            dispatch
                .inner
                .printer
                .upgrade()
                .expect("the printer is alive")
                .get_state_message()
                .message,
            "Shutdown due to M112 command"
        );
    }

    #[test]
    fn test_help_lists_the_active_commands() {
        let (dispatch, output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, _) = recorder();
        dispatch
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();

        run(&dispatch, "HELP");

        let lines = emitted(&output);
        assert!(
            lines
                .iter()
                .any(|l| l.contains("SET_PIN") && l.contains("Set a pin")),
            "{lines:?}"
        );
    }

    #[test]
    fn test_help_before_ready_names_the_base_commands() {
        // Not ready: only the base commands are active, and the list says so.
        let (dispatch, output) = dispatch();
        let (handler, _) = recorder();
        dispatch
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();

        run(&dispatch, "HELP");

        let text = emitted(&output).join("\n");
        assert!(text.contains("Printer is not ready"), "{text}");
        assert!(!text.contains("SET_PIN"), "{text}");
        // `RESTART` is a base command and carries help text.
        assert!(text.contains("RESTART"), "{text}");
    }

    #[test]
    fn test_restart_prepares_the_toolhead_and_fires_the_event() {
        struct TestHooks {
            log: Arc<Mutex<Vec<String>>>,
            print_time: f64,
        }
        impl crate::core::klippy::printer::RestartHooks for TestHooks {
            fn get_last_move_time(&self) -> f64 {
                self.log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("get_last_move_time".to_string());
                self.print_time
            }
            fn dwell(&self, _delay: f64) {
                self.log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("dwell".to_string());
            }
            fn wait_moves(&self) {
                self.log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("wait_moves".to_string());
            }
        }

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        let log = Arc::new(Mutex::new(Vec::new()));
        printer.register_restart_hooks(Arc::new(TestHooks {
            log: Arc::clone(&log),
            print_time: 1.5,
        }));
        let seen = Arc::new(Mutex::new(Vec::new()));
        {
            let seen = Arc::clone(&seen);
            printer.register_event_handler(
                KlippyEvent::GcodeRequestRestart { print_time: 0.0 },
                Box::new(move |event| {
                    if let KlippyEvent::GcodeRequestRestart { print_time } = event {
                        seen.lock()
                            .unwrap_or_else(|p| p.into_inner())
                            .push(*print_time);
                    }
                }),
            );
        }
        // Only a ready printer reaches the toolhead.
        printer.send_event(&KlippyEvent::KlippyReady);

        dispatch.run_script_sync("RESTART").unwrap();

        assert_eq!(printer.run(), "restart");
        assert_eq!(
            *log.lock().unwrap_or_else(|p| p.into_inner()),
            ["get_last_move_time", "dwell", "wait_moves"]
        );
        assert_eq!(*seen.lock().unwrap_or_else(|p| p.into_inner()), [1.5]);
    }

    #[test]
    fn test_echo_uses_the_info_prefix() {
        let (dispatch, output) = dispatch();

        // Extended commands need `KEY=VALUE` parameters, so a bare word is a
        // malformed command (upstream is the same); the line is echoed with the
        // `// ` prefix either way once it parses.
        run(&dispatch, "ECHO MESSAGE=hi");

        assert_eq!(emitted(&output), ["// ECHO MESSAGE=hi"]);
    }

    // -----------------------------------------------------------------------
    // Mux commands
    // -----------------------------------------------------------------------

    /// Register two values of a mux command and return what each saw.
    fn mux_dispatch() -> (Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>) {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let seen = Arc::new(Mutex::new(Vec::new()));
        for value in ["fan", "light"] {
            let seen = Arc::clone(&seen);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                seen.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(format!("{value}={}", gcmd.get_float("VALUE")?));
                Ok(())
            });
            dispatch
                .register_mux_command("SET_PIN", "PIN", Some(value), handler, Some("Set a pin"))
                .unwrap();
        }
        (dispatch, seen)
    }

    #[test]
    fn test_a_mux_command_dispatches_on_its_key() {
        let (dispatch, seen) = mux_dispatch();

        run(&dispatch, "SET_PIN PIN=light VALUE=0.5");

        assert_eq!(*seen.lock().unwrap(), ["light=0.5"]);
    }

    #[test]
    fn test_a_mux_value_that_is_not_registered_names_the_options() {
        let (dispatch, _seen) = mux_dispatch();

        let err = dispatch
            .run_script_sync("SET_PIN PIN=nope VALUE=1")
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "The value 'nope' is not valid for PIN. Options: 'fan', 'light'"
        );
    }

    #[test]
    fn test_a_mux_command_may_have_only_one_key() {
        let (dispatch, _seen) = mux_dispatch();
        let (handler, _) = recorder();

        let err = dispatch
            .register_mux_command("SET_PIN", "OTHER", Some("x"), handler, None)
            .unwrap_err();

        assert!(err.contains("may have only one key (PIN)"), "{err}");
    }

    #[test]
    fn test_a_mux_command_without_the_key_uses_the_default_value() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let seen = Arc::new(Mutex::new(Vec::new()));
        {
            let seen = Arc::clone(&seen);
            let default: CommandHandler = sync(move |_| {
                seen.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("default".to_string());
                Ok(())
            });
            dispatch
                .register_mux_command("MY_MUX", "PIN", None, default, None)
                .unwrap();
        }
        {
            let seen = Arc::clone(&seen);
            let named: CommandHandler = sync(move |_| {
                seen.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("named".to_string());
                Ok(())
            });
            dispatch
                .register_mux_command("MY_MUX", "PIN", Some("fan"), named, None)
                .unwrap();
        }

        // No key: the `None` value handles it.
        run(&dispatch, "MY_MUX");
        assert_eq!(*seen.lock().unwrap(), ["default"]);

        // A registered key: the named handler.
        run(&dispatch, "MY_MUX PIN=fan");
        assert_eq!(*seen.lock().unwrap(), ["default", "named"]);

        // An unknown key is still reported against the named options.
        let err = dispatch.run_script_sync("MY_MUX PIN=nope").unwrap_err();
        assert!(err.to_string().contains("is not valid for PIN"), "{err}");
    }

    // -----------------------------------------------------------------------
    // Parameter accessors
    // -----------------------------------------------------------------------

    /// Build a command with `params` for the accessors.
    fn command(line: &str) -> GcodeCommand {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(printer);
        let parsed = parse_line(line).unwrap();
        let params = if parsed.traditional {
            parsed.params
        } else {
            parse_extended(&parsed.raw_params, &parsed.commandline).unwrap()
        };
        GcodeCommand {
            dispatch: dispatch.inner,
            command: parsed.command,
            commandline: parsed.commandline,
            params,
            need_ack: AtomicBool::new(false),
        }
    }

    #[test]
    fn test_a_missing_parameter_names_the_line() {
        let gcmd = command("MY_CMD A=1");

        let err = gcmd.get_str("B").unwrap_err();

        assert_eq!(err.to_string(), "Error on 'MY_CMD A=1': missing B");
    }

    #[test]
    fn test_an_unparseable_parameter_is_reported() {
        let gcmd = command("MY_CMD A=abc");

        let err = gcmd.get_int("A").unwrap_err();

        assert_eq!(
            err.to_string(),
            "Error on 'MY_CMD A=abc': unable to parse abc"
        );
    }

    #[test]
    fn test_defaults_and_ranges() {
        let gcmd = command("MY_CMD A=0.5");

        assert_eq!(gcmd.get_float("A").unwrap(), 0.5);
        assert_eq!(gcmd.get_float_default("B", 2.0).unwrap(), 2.0);
        assert_eq!(gcmd.get_int_default("C", 7).unwrap(), 7);
        assert_eq!(gcmd.get_float_range("A", 0.0, 1.0).unwrap(), 0.5);

        let err = gcmd.get_float_range("A", 1.0, 2.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Error on 'MY_CMD A=0.5': A must have minimum of 1"
        );
    }

    #[test]
    fn test_bounds_and_the_generic_getter() {
        let gcmd = command("MY_CMD A=5 B=0.5 C=-1");

        // Upstream's wording for every bound.
        assert_eq!(gcmd.get_int_bounded("A", Some(0), Some(10)).unwrap(), 5);
        assert_eq!(
            gcmd.get_int_bounded("C", Some(0), None)
                .unwrap_err()
                .to_string(),
            "Error on 'MY_CMD A=5 B=0.5 C=-1': C must have minimum of 0"
        );
        assert_eq!(
            gcmd.get_int_bounded("A", None, Some(4))
                .unwrap_err()
                .to_string(),
            "Error on 'MY_CMD A=5 B=0.5 C=-1': A must have maximum of 4"
        );
        // `above`/`below` are strict: equal is already out of bounds.
        assert!(gcmd.get_float_bounded("B", Some(0.5), None).is_err());
        assert_eq!(gcmd.get_float_bounded("B", Some(0.4), None).unwrap(), 0.5);
        assert!(gcmd.get_float_bounded("B", None, Some(0.5)).is_err());

        // The generic getter takes a parser and the same bounds.
        let parsed = gcmd
            .get("A", None, |v| v.parse::<u32>().ok(), None, None, None, None)
            .unwrap();
        assert_eq!(parsed, 5);
        // An absent parameter uses the default.
        assert_eq!(
            gcmd.get("MISSING", Some(7), parse_int, None, None, None, None)
                .unwrap(),
            7
        );
    }

    #[test]
    fn test_raw_command_parameters_skip_a_line_number_and_checksum() {
        // The command is at the head of the line: the text after it.
        let gcmd = command("SET_PIN PIN=fan VALUE=1");
        assert_eq!(gcmd.get_raw_command_parameters(), "PIN=fan VALUE=1");

        // A line-numbered line drops the number and a trailing checksum.
        let gcmd = command("N5 M110 X1*45");
        assert_eq!(gcmd.get_raw_command_parameters(), "X1");

        // No parameters: empty.
        let gcmd = command("M110");
        assert_eq!(gcmd.get_raw_command_parameters(), "");
    }

    #[test]
    fn test_a_command_can_be_created_without_parsing() {
        let (dispatch, _output) = dispatch();
        let params = HashMap::from([("PIN".to_string(), "fan".to_string())]);

        let gcmd = dispatch.create_gcode_command("SET_PIN", "SET_PIN PIN=fan", params);

        assert_eq!(gcmd.command(), "SET_PIN");
        assert_eq!(gcmd.commandline(), "SET_PIN PIN=fan");
        assert_eq!(gcmd.get_str("PIN").unwrap(), "fan");
        // A synthesised command is never acknowledged.
        assert!(!gcmd.ack(None));
    }

    #[test]
    fn test_run_script_from_command_runs_a_script() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, seen) = recorder();
        dispatch
            .register_command("MY_CMD", handler, None, false)
            .unwrap();

        dispatch.run_script_from_command_sync("MY_CMD A=1").unwrap();

        assert_eq!(*seen.lock().unwrap(), ["MY_CMD A=1"]);
    }

    #[test]
    fn test_a_command_error_fires_the_command_error_event() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch.inner.set_ready(true);
        let fired = Arc::new(AtomicBool::new(false));
        {
            let fired = Arc::clone(&fired);
            printer.register_event_handler(
                KlippyEvent::GcodeCommandError,
                Box::new(move |_| fired.store(true, Ordering::SeqCst)),
            );
        }
        dispatch
            .register_command(
                "FAIL",
                sync(|_| Err(CommandError::new("boom"))),
                None,
                false,
            )
            .unwrap();

        let err = dispatch.run_script_sync("FAIL").unwrap_err();

        assert_eq!(err.to_string(), "boom");
        assert!(fired.load(Ordering::SeqCst));
    }

    #[test]
    fn test_a_panic_does_not_fire_the_command_error_event() {
        // Upstream fires `gcode:command_error` only for a `CommandError`; a
        // panic is an internal error and shuts the printer down instead
        // (`klippy/gcode.py:223-234`).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch.inner.set_ready(true);
        let fired = Arc::new(AtomicBool::new(false));
        {
            let fired = Arc::clone(&fired);
            printer.register_event_handler(
                KlippyEvent::GcodeCommandError,
                Box::new(move |_| fired.store(true, Ordering::SeqCst)),
            );
        }
        dispatch
            .register_command("BOOM", sync(|_| panic!("handler bug")), None, false)
            .unwrap();

        let _ = dispatch.run_script_sync("BOOM");

        assert!(!fired.load(Ordering::SeqCst));
    }

    #[tokio::test]
    async fn test_an_acknowledged_line_does_not_stop_the_script_on_error() {
        // The file/serial protocol (`need_ack`) reports the error, fires the
        // event and acks the line instead of propagating it
        // (`klippy/gcode.py:223-228`), so the rest of the script still runs.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch.inner.set_ready(true);
        let calls = Arc::new(AtomicUsize::new(0));
        {
            let calls = Arc::clone(&calls);
            dispatch
                .register_command(
                    "FAIL",
                    sync(move |_| {
                        calls.fetch_add(1, Ordering::SeqCst);
                        Err(CommandError::new("boom"))
                    }),
                    None,
                    false,
                )
                .unwrap();
        }
        dispatch
            .register_command("AFTER", sync(|_| Ok(())), None, false)
            .unwrap();
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            dispatch.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }

        process_line(&dispatch.inner, "FAIL", true).await.unwrap();
        process_line(&dispatch.inner, "AFTER", true).await.unwrap();

        assert_eq!(calls.load(Ordering::SeqCst), 1);
        let lines = output.lock().unwrap_or_else(|p| p.into_inner()).clone();
        assert_eq!(lines, ["!! boom", "ok", "ok"]);
    }

    #[test]
    fn test_ack_does_nothing_for_an_api_line() {
        let gcmd = command("M115");

        // The only `need_ack` producer is the file/serial input protocol, which
        // this host does not have, so an API line is never acknowledged.
        assert!(!gcmd.ack(None));
        assert!(!gcmd.ack(Some("T:0")));
    }

    #[test]
    fn test_the_status_reports_the_command_table() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, _) = recorder();
        dispatch
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();

        let status = dispatch.get_status(0.0);

        assert_eq!(status["commands"]["SET_PIN"]["help"], "Set a pin");
        assert!(status["commands"]["M110"].is_object());
    }

    #[test]
    fn test_the_status_before_ready_lists_only_the_base_commands() {
        let (dispatch, _output) = dispatch();
        let (handler, _) = recorder();
        dispatch
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();

        let status = dispatch.get_status(0.0);

        assert!(status["commands"]["M110"].is_object());
        assert!(status["commands"]["SET_PIN"].is_null());
    }

    #[test]
    fn test_a_declared_parameter_list_reaches_the_status() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        dispatch
            .register_command_with_params(
                "MY_CMD",
                sync(|_| Ok(())),
                Some("My command"),
                &["VALUE", "SPEED"],
                false,
            )
            .unwrap();

        let status = dispatch.get_status(0.0);

        // Declaration order, and alongside the help text.
        assert_eq!(
            status["commands"]["MY_CMD"]["parameters"],
            json!(["VALUE", "SPEED"])
        );
        assert_eq!(status["commands"]["MY_CMD"]["help"], "My command");
    }

    #[test]
    fn test_a_mux_command_reports_its_key_first_and_once() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        dispatch
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some("fan"),
                sync(|_| Ok(())),
                Some("Set a pin"),
                // The key is declared by the mux itself: a caller that repeats
                // it (twice) does not produce a duplicate.
                &["VALUE", "PIN", "VALUE"],
            )
            .unwrap();

        let status = dispatch.get_status(0.0);

        assert_eq!(
            status["commands"]["SET_PIN"]["parameters"],
            json!(["PIN", "VALUE"])
        );
    }

    #[test]
    fn test_a_mux_command_reports_the_parameters_of_every_value() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        dispatch
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some("fan"),
                sync(|_| Ok(())),
                Some("Set a pin"),
                &["VALUE"],
            )
            .unwrap();
        // `[pwm_cycle_time]`: the same command and key, another value, and a
        // name no earlier value declares.
        dispatch
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some("tool"),
                sync(|_| Ok(())),
                None,
                &["VALUE", "CYCLE_TIME"],
            )
            .unwrap();

        let status = dispatch.get_status(0.0);

        // The union: key first, each name once, in the order they first appeared.
        assert_eq!(
            status["commands"]["SET_PIN"]["parameters"],
            json!(["PIN", "VALUE", "CYCLE_TIME"])
        );
    }

    #[test]
    fn test_a_mux_value_that_declares_nothing_keeps_the_parameters() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        dispatch
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some("fan"),
                sync(|_| Ok(())),
                None,
                &["VALUE"],
            )
            .unwrap();
        dispatch
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some("led"),
                sync(|_| Ok(())),
                None,
                &[],
            )
            .unwrap();

        let status = dispatch.get_status(0.0);

        // A later value's silence is not a withdrawal.
        assert_eq!(
            status["commands"]["SET_PIN"]["parameters"],
            json!(["PIN", "VALUE"])
        );
    }

    #[test]
    fn test_a_mux_value_cannot_be_registered_twice() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        dispatch
            .register_mux_command("SET_PIN", "PIN", Some("fan"), sync(|_| Ok(())), None)
            .unwrap();

        let err = dispatch
            .register_mux_command("SET_PIN", "PIN", Some("fan"), sync(|_| Ok(())), None)
            .unwrap_err();

        assert!(err.contains("already registered"), "{err}");
    }

    #[test]
    fn test_a_command_without_declared_parameters_has_no_parameters_key() {
        let (dispatch, _output) = dispatch();
        dispatch.inner.set_ready(true);
        let (handler, _) = recorder();
        dispatch
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();

        let status = dispatch.get_status(0.0);

        // The shape a client sees today, unchanged: `help`, and nothing else.
        assert_eq!(status["commands"]["SET_PIN"]["help"], "Set a pin");
        assert!(status["commands"]["SET_PIN"].get("parameters").is_none());
        // A built-in that declares nothing is the same.
        assert!(status["commands"]["M110"].get("parameters").is_none());
    }

    #[test]
    fn test_a_handler_that_panics_shuts_the_printer_down() {
        // Upstream's bare `except:` around a handler (`klippy/gcode.py:230-234`):
        // anything that is not a `CommandError` is klippy's own bug, so the
        // printer is halted with the command named.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch.inner.set_ready(true);
        dispatch
            .register_command("BOOM", sync(|_| panic!("handler bug")), None, false)
            .unwrap();

        let err = dispatch.run_script_sync("BOOM").unwrap_err();

        assert_eq!(err.to_string(), "Internal error on command:\"BOOM\"");
        assert_eq!(
            printer.get_state_message().category,
            crate::core::klippy::printer::PrinterState::Shutdown
        );
    }

    #[test]
    fn test_a_command_error_does_not_shut_the_printer_down() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let dispatch = GCodeDispatch::new(Arc::clone(&printer));
        dispatch.inner.set_ready(true);
        dispatch
            .register_command(
                "BAD",
                sync(|_| Err(CommandError::new("bad parameter"))),
                None,
                false,
            )
            .unwrap();

        let err = dispatch.run_script_sync("BAD").unwrap_err();

        assert_eq!(err.to_string(), "bad parameter");
        assert_ne!(
            printer.get_state_message().category,
            crate::core::klippy::printer::PrinterState::Shutdown
        );
    }
}
