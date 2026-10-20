//! `template_evaluator` — the shared display-template evaluator.
//!
//! Upstream's `PrinterTemplateEvaluator` (`klippy/extras/output_pin.py:94-193`):
//! a printer-level singleton that renders `[display_template]` sections on a
//! 0.5 s reactor timer and hands each rendered text to the command that
//! subscribed it. The subscribers are `SET_PIN TEMPLATE=`
//! ([`output_pin`](super::output_pin)) and `SET_FAN_SPEED TEMPLATE=`
//! ([`fan_generic`](super::fan_generic)); they differ only in what their
//! callback does with the text.
//!
//! # Timing
//!
//! A template is **not** evaluated at lookahead or flush time. Upstream
//! registers one reactor timer with `reactor.NOW` when a subscription appears
//! (`_activate_timer`, `output_pin.py:111-114`), renders every active template
//! on each firing, and stops the timer once nothing is subscribed
//! (`_render`'s empty-table branch, `:131-136`). The 0.5 s figure is
//! [`RENDER_TIME`]. Each render reaches the pin/fan immediately —
//! `send_async_request` dated with the *estimated* print time
//! (`output_pin.py:68-90`), which is why the subscriber side does not wait for
//! a move.
//!
//! # Subscription
//!
//! [`PrinterTemplateEvaluator::set_template`] is upstream's `set_template`
//! (`output_pin.py:164-184`): it reads `TEMPLATE=` plus every `PARAM_*` from
//! the command and either subscribes the callback under a key or — when
//! `TEMPLATE=` is empty — unsubscribes that key. Upstream keys the table by the
//! consumer's bound method `_template_update` (`:128`), which is one object per
//! pin or fan; a Rust closure has no identity of its own, so the consumer names
//! itself instead (`"output_pin fan"`, `"fan_generic side_fan"`). One
//! subscription per consumer follows in both spellings.
//!
//! The three refusals are upstream's wording: an unknown template name, a
//! `PARAM_*` the template does not declare, and a `PARAM_*` value that is not
//! a literal ([`parse_literal`](super::led::parse_literal), the port of
//! `ast.literal_eval`).
//!
//! # What is not here
//!
//! * **`SET_LED_TEMPLATE`.** The LED command that is the third subscriber
//!   upstream (`led.py:100-135`) is not registered in this port yet, so
//!   nothing but `SET_PIN` and `SET_FAN_SPEED` subscribes.
//! * **The `render` template helper.** `_render` binds
//!   `context['render'] = render` (`output_pin.py:137-140`) so a template may
//!   render another display template by name. This port's engine binds the
//!   callables [`Builtin`] knows ([`template`](super::template)), and `render`
//!   is not one of them: a template that calls it fails the render, which is
//!   logged and drives `0.` like any other render failure.
//! * **`action_emergency_stop` / `action_call_remote_method`.** The context is
//!   the three globals `gcode_macro.create_template_context` binds plus
//!   `printer` (`gcode_macro.py:106-113`); the two remaining `action_*` names
//!   upstream binds are not in this port's [`Builtin`] set, so a template that
//!   calls them fails its render the same way.
//! * **The per-tick render cache.** Upstream reuses a render across
//!   subscriptions with identical `(template, params)` (`output_pin.py:143-156`);
//!   here every subscription renders its own text.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::extras::led::{lookup_display_templates, parse_literal, DisplayTemplates};
use crate::core::klippy::extras::template::{Builtin, Context, PrinterView, Rt};
use crate::core::klippy::gcode::{CommandError, GcodeCommand};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

/// Time between template re-renders, in seconds (upstream's `RENDER_TIME`,
/// `output_pin.py:98`).
pub const RENDER_TIME: f64 = 0.500;

/// The printer object name upstream's lazy lookup registers
/// (`output_pin.py:190`).
pub const TEMPLATE_EVALUATOR_OBJECT: &str = "template_evaluator";

/// One subscription's receiving end: upstream's bound `_template_update`
/// (`output_pin.py:245-250`, `fan_generic.py:27-32`).
pub type TemplateUpdate = Arc<dyn Fn(&str) + Send + Sync>;

/// The shared evaluator: upstream's `PrinterTemplateEvaluator`.
///
/// Every consumer asks [`lookup_template_eval`] for it, so there is one timer
/// and one table however many pins and fans subscribe.
pub struct PrinterTemplateEvaluator {
    core: Arc<EvaluatorCore>,
}

/// What the timer callback and the commands share.
///
/// Behind its own `Arc` because the timer's callback needs a handle on it that
/// refers back weakly: the callback lives in the reactor, and a strong
/// reference from there would keep the machine's own object graph alive.
struct EvaluatorCore {
    /// For the reactor and the `printer` template global.
    printer: Arc<Printer>,
    /// The `[display_template]` registry (`led.rs`'s, upstream's
    /// `lookup_display_templates`).
    templates: Arc<DisplayTemplates>,
    /// The active subscriptions, keyed by consumer name.
    active: Mutex<BTreeMap<String, Active>>,
    /// The render timer, present exactly while something is subscribed.
    timer: Mutex<Option<TimerHandle>>,
}

/// One active subscription (upstream's `active_templates` value).
struct Active {
    /// The `[display_template <name>]` section name.
    template: String,
    /// The command's `PARAM_*` values, keyed by the templates' own option
    /// names (`param_myvar` and so on), already parsed as literals.
    lparams: Vec<(String, Value)>,
    /// Where the rendered text goes.
    callback: TemplateUpdate,
}

/// Look up the shared evaluator, creating it on first use.
///
/// Upstream's `lookup_template_eval` (`output_pin.py:187-193`): the object
/// exists only once a consumer asks for it, and every consumer then shares it.
///
/// # Errors
/// Only if the object name is already taken, which a duplicate lazy creation
/// would mean, or if the `[display_template]` registry cannot be created.
pub fn lookup_template_eval(
    printer: &Arc<Printer>,
) -> Result<Arc<PrinterTemplateEvaluator>, ConfigError> {
    if let Some(existing) =
        printer.lookup_object_as::<PrinterTemplateEvaluator>(TEMPLATE_EVALUATOR_OBJECT)
    {
        return Ok(existing);
    }
    let created = Arc::new(PrinterTemplateEvaluator {
        core: Arc::new(EvaluatorCore {
            printer: Arc::clone(printer),
            templates: lookup_display_templates(printer)?,
            active: Mutex::new(BTreeMap::new()),
            timer: Mutex::new(None),
        }),
    });
    printer.add_object(
        TEMPLATE_EVALUATOR_OBJECT,
        Arc::clone(&created) as Arc<dyn PrinterObject>,
    )?;
    Ok(created)
}

impl PrinterTemplateEvaluator {
    /// Upstream's `set_template` (`output_pin.py:164-184`): subscribe
    /// `callback` to the command's `TEMPLATE=` (with its `PARAM_*` values), or
    /// unsubscribe `key` when that word is empty.
    ///
    /// `key` is the consumer's stable identity. Upstream's table is keyed by
    /// the consumer's bound `_template_update`, one object per pin or fan; this
    /// is that object's name (`"output_pin fan"`), so a second subscription on
    /// the same consumer replaces the first.
    ///
    /// An empty `TEMPLATE=` is not a missing one: upstream reads the word and
    /// branches on its truthiness, so `SET_PIN PIN=x TEMPLATE=` cancels the
    /// pin's subscription and moves nothing.
    ///
    /// # Errors
    /// Upstream's three refusals, in this order: an unknown template name, a
    /// `PARAM_*` the template does not declare, a `PARAM_*` value that is not
    /// a literal. Plus "missing TEMPLATE", which upstream's `gcmd.get` raises
    /// and its callers never trigger.
    pub fn set_template(
        &self,
        key: &str,
        gcmd: &GcodeCommand,
        callback: TemplateUpdate,
    ) -> Result<(), CommandError> {
        let Some(tpl_name) = gcmd.get_command_parameters().get("TEMPLATE").cloned() else {
            return Err(CommandError::new("missing TEMPLATE"));
        };

        if tpl_name.is_empty() {
            let emptied = {
                let mut active = self.core.lock_active();
                active.remove(key).is_some() && active.is_empty()
            };
            if emptied {
                self.stop_timer();
            }
            return Ok(());
        }

        let declared =
            self.core.templates.param_names(&tpl_name).ok_or_else(|| {
                CommandError::new(format!("Unknown display_template '{tpl_name}'"))
            })?;
        let mut lparams = Vec::new();
        for (name, value) in gcmd.get_command_parameters() {
            if !name.starts_with("PARAM_") {
                continue;
            }
            // Upstream lowercases the whole option name, prefix included
            // (`output_pin.py:173-175`), so `PARAM_MYVAR` names `param_myvar`.
            let param = name.to_ascii_lowercase();
            if !declared.contains(&param) {
                return Err(CommandError::new(format!(
                    "Invalid display_template parameter: {param}"
                )));
            }
            let literal = parse_literal(value).ok_or_else(|| {
                CommandError::new(format!("Unable to parse '{value}' as a literal"))
            })?;
            lparams.push((param, literal));
        }

        self.core.lock_active().insert(
            key.to_string(),
            Active {
                template: tpl_name,
                lparams,
                callback,
            },
        );
        self.activate_timer();
        Ok(())
    }

    /// One timer period's worth of rendering, without the timer: the `eventtime`
    /// is dated the way the reactor would date it, and the answer is the next
    /// wake time, or `None` when nothing is subscribed.
    ///
    /// This is upstream's `_render` (`output_pin.py:131-163`), reachable here
    /// without a reactor so a unit test can step a period by hand instead of
    /// waiting for one.
    #[cfg(test)]
    pub(crate) fn render_once(&self, eventtime: f64) -> Option<f64> {
        self.core.render_once(eventtime)
    }

    /// Start the render timer, unless one runs or nothing is subscribed
    /// (upstream `_activate_timer`, `output_pin.py:111-114`).
    fn activate_timer(&self) {
        if self.core.lock_active().is_empty() {
            return;
        }
        let mut timer = self.core.lock_timer();
        if timer.is_some() {
            return;
        }
        let core = Arc::downgrade(&self.core);
        let handle = self.core.printer.reactor().register_timer_named(
            "template_evaluator",
            Box::new(move |eventtime| {
                let Some(core) = Weak::upgrade(&core) else {
                    return None;
                };
                core.render_once(eventtime)
            }),
            // `reactor.NOW`: the first render happens on the next dispatch
            // round, not one period from now (`output_pin.py:114`).
            0.0,
        );
        *timer = Some(handle);
    }

    /// Stop the render timer (upstream `self.reactor.unregister_timer`,
    /// `output_pin.py:133-135`).
    fn stop_timer(&self) {
        let handle = self.core.lock_timer().take();
        if let Some(handle) = handle {
            handle.cancel();
        }
    }
}

impl EvaluatorCore {
    /// Upstream's `_render` (`output_pin.py:131-163`): render every
    /// subscription and hand it the text; `None` once nothing is subscribed.
    fn render_once(&self, eventtime: f64) -> Option<f64> {
        // Taken out from under the lock so a subscription's callback runs with
        // the table free: a callback drives a pin or a fan, which takes that
        // consumer's own locks, and holding the table across it would put an
        // unrelated lock inside this one.
        let snapshot: Vec<ActiveView> = {
            let active = self.lock_active();
            active
                .values()
                .map(|entry| ActiveView {
                    template: entry.template.clone(),
                    lparams: entry.lparams.clone(),
                    callback: Arc::clone(&entry.callback),
                })
                .collect()
        };
        if snapshot.is_empty() {
            // Nothing to render: retire the timer. The caller is the timer
            // itself, so dropping the handle is all that is left to do.
            self.lock_timer().take();
            return None;
        }
        for entry in snapshot {
            let text = self.render_template(&entry.template, &entry.lparams);
            (entry.callback)(&text);
        }
        Some(eventtime + RENDER_TIME)
    }

    /// One template's text for this period: its declared `param_*` values with
    /// the command's own on top, or `""` when rendering fails.
    ///
    /// A failure is upstream's `display template render error`: logged, and
    /// the subscription's callback still runs with an empty text, which is
    /// what makes it drive `0.` rather than skip the period
    /// (`output_pin.py:148-152`).
    fn render_template(&self, name: &str, lparams: &[(String, Value)]) -> String {
        let mut context = self.context();
        match self.templates.render(name, &mut context, lparams) {
            Some(Ok(text)) => text,
            Some(Err(err)) => {
                warn!("display template render error: {err}");
                String::new()
            }
            // The registry never drops a template, so a name that resolved when
            // it was subscribed still resolves here.
            None => String::new(),
        }
    }

    /// The globals a `[display_template]` body renders against: upstream's
    /// `create_template_context` (`gcode_macro.py:106-113`), without the
    /// `params`/`rawparams` a g-code macro adds — a template has no command
    /// line, only its `param_*` values, which
    /// [`DisplayTemplates::render`](super::led::DisplayTemplates::render)
    /// binds.
    fn context(&self) -> Context {
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
        context
    }

    /// The subscriptions; a poisoned lock is recovered from so one panicking
    /// callback cannot wedge the evaluator (repo pattern, `adc_scaled.rs`).
    fn lock_active(&self) -> MutexGuard<'_, BTreeMap<String, Active>> {
        self.active
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The timer slot, with the same poison recovery as [`Self::lock_active`].
    fn lock_timer(&self) -> MutexGuard<'_, Option<TimerHandle>> {
        self.timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

/// A subscription copied out of the table for one render pass.
struct ActiveView {
    template: String,
    lparams: Vec<(String, Value)>,
    callback: TemplateUpdate,
}

impl PrinterObject for PrinterTemplateEvaluator {
    /// The subscribed template names. Upstream's object has no `get_status` at
    /// all (`output_pin.py:99-108`); the trait needs one, and this is the data
    /// it would report.
    fn get_status(&self, _eventtime: f64) -> Value {
        let active = self.core.lock_active();
        let names: Vec<&str> = active
            .values()
            .map(|entry| entry.template.as_str())
            .collect();
        json!({ "templates": names })
    }

    /// Not client-visible, matching upstream's `get_status`-less object.
    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for PrinterTemplateEvaluator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let active = self.core.lock_active();
        f.debug_struct("PrinterTemplateEvaluator")
            .field("templates", &active.len())
            .finish_non_exhaustive()
    }
}
