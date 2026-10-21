//! `[menu ...]` — the LCD menu engine (upstream
//! `klippy/extras/display/menu.py` and `menu_keys.py`).
//!
//! A menu is a tree of [`MenuItem`]s built from two sources, in upstream's
//! order: the shipped [`MENU_CFG`] (a copy of
//! `klippy/extras/display/menu.cfg`) and then every `[menu <name>]` section of
//! the main config, where a section the shipped file also defines **replaces**
//! it (`MenuManager.add_menuitem`, `menu.py:1004-1020`). An item is named by its
//! namespace — the section's sub-name, `__main __tune` — and hangs under its
//! parent by that name: the item whose namespace is one name shorter is its
//! parent.
//!
//! | piece | upstream | what it is |
//! |---|---|---|
//! | [`MenuManager`] | `MenuManager` (`menu.py:586`) | the stack, the key events, the script queue, the `menu` object |
//! | [`MenuItem`] | `MenuElement` and its subclasses (`menu.py:17`) | one item: `disabled`, `command`, `input`, `list`, `vsdlist` |
//! | [`MenuRow`] | `MenuList.draw_container` (`menu.py:499`) | one laid-out screen row |
//! | `menu_keys` | `menu_keys.py` | the key names, and the buttons that send them |
//!
//! # Templates
//!
//! Every scripted option is a `gcode_macro` template evaluated in the same
//! environment as a macro body ([`super::super::template`]) with the same
//! `printer` view and actions, plus `menu`: `eventtime`, `ns`, `event` and — in
//! an input item — `input`. A rendered script is **queued**, not run in place
//! (`MenuManager.queue_gcode` / `dispatch_gcode`, `menu.py:943-963`): the first
//! script scheduled arms one reactor callback that drains the queue, so the
//! scripts of one key press run in order and none runs inside the refresh that
//! produced it.
//!
//! An `enable` option that is not a boolean is a template too, and its rendered
//! text is read as a Python literal (`ast.literal_eval`, `menu.py:113-118`):
//! `{printer.idle_timeout.state == "Printing"}` renders `True`/`False` and the
//! item is enabled on the first.
//!
//! # What is not here
//!
//! * **Drawing onto the panel.** The framework has no `draw_text` (see
//!   `display.rs`'s module docs), so the rows a container lays out are
//!   [`MenuManager::screen_rows`] and nothing paints them: the state machine,
//!   the viewport and the scroller run, the panel keeps its blank screen. This
//!   is the gap `display_data` has as well.
//! * **The `menu:*` printer events** (`begin`, `init`, `populate`, `exit`).
//!   This host's event bus is a generated enum and no part subscribes to them.
//! * **`menu.back()` / `menu.exit()` inside a script** (`menu.py:679-684`): a
//!   script context here carries plain values, so those two callables are
//!   absent. `menu.cfg` never calls them.
//! * **The rotary encoder.** `menu_keys.py:19-48` calls
//!   `buttons.register_rotary_encoder`; this host's `buttons` registry has no
//!   encoder, so `encoder_pins` is accepted and reported as unwired. The key
//!   pins are registered as press buttons; an analog range on one is reported
//!   rather than silently read as a digital input.
//! * **`menuitem_from`** — building an item from keywords instead of a config
//!   section. Upstream uses it for the list's `..` item and the SD-card files
//!   only; here those are [`Element::Back`] and [`MenuManager::command_item`].

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Map, Value};

use crate::core::klippy::config::{Config, ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::buttons::PrinterButtons;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::py_literal;
use crate::core::klippy::extras::template::{Builtin, Context, PrinterView, Rt, Template};
use crate::core::klippy::extras::virtual_sdcard::VirtualSdCard;
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::{Reactor, TimerHandle};

use super::display::{MenuOptions, PrinterLCD};

/// The shipped menu layout (`klippy/extras/display/menu.cfg`).
///
/// Vendored verbatim from the upstream submodule; the drift guard test keeps it
/// that way.
pub const MENU_CFG: &str = include_str!("menu.cfg");

/// The name the shipped layout is reported under. There is no path at run time.
const MENU_CFG_NAME: &str = "menu.cfg";

/// The name the menu registers as (`printer.add_object('menu', self)`).
pub const MENU_OBJECT: &str = "menu";

/// The virtual SD card object a `vsdlist` lists (`MenuVSDList._populate`).
const VIRTUAL_SDCARD_OBJECT: &str = "virtual_sdcard";

/// How often the manager's own timer runs (`menu.py:670`).
const TIMER_DELAY: f64 = 1.0;

/// Seconds between two scroller steps (`menu.py:120`).
const SCROLL_DELAY: f64 = 0.5;

/// Seconds an input's change waits before its `gcode` runs (`menu.py:477`).
const INPUT_CHANGE_DELAY: f64 = 0.250;

/// How long a click must be held to become a `long_click`
/// (`menu_keys.py:10`).
const LONG_PRESS_DURATION: f64 = 0.800;

/// The item types a `type` option may name (`menu.py:578-584`).
const MENU_ITEM_TYPES: &[&str] = &["disabled", "command", "input", "list", "vsdlist"];

/// The button options `menu_keys.py` registers, with the key each sends.
const KEY_PINS: &[(&str, &str)] = &[
    ("click_pin", "click"),
    ("back_pin", "back"),
    ("up_pin", "up"),
    ("down_pin", "down"),
];

/// A failure in the menu engine, as the text a log line or a config error
/// carries.
type MenuError = String;

// ===========================================================================
// Item description: the options every item reads
// ===========================================================================

/// What an item's `enable` option came out as: a boolean, or a template whose
/// rendered text is read as a Python literal (`menu.py:110-118`).
enum Enable {
    Bool(bool),
    Template(Arc<Template>),
}

/// What an item's `name` option came out as (`menu.py:49-53`).
enum Name {
    /// No template: the name as written (the list's `..` item).
    Literal(String),
    Template(Arc<Template>),
}

/// The rolling state of a long name (`menu.py:76-100`, `:148-165`).
#[derive(Default)]
struct Scroll {
    /// Where the name is cut, or `None` while it is whole.
    pos: Option<usize>,
    /// What the next update does: `Some(true)` steps, `Some(false)` holds,
    /// `None` resets (`need_scroller`).
    request: Option<bool>,
    /// The earliest time the next step may run.
    next: f64,
    /// The item's last heartbeat.
    last_heartbeat: Option<f64>,
}

/// One `input` item's value and bounds (`MenuInput`, `menu.py:424-534`).
struct InputData {
    /// `input` — a required template upstream (`load_template(config, 'input')`).
    value_tpl: Arc<Template>,
    /// `input_min`, defaulting to `-999999.0`.
    min_tpl: Arc<Template>,
    /// `input_max`, defaulting to `999999.0`.
    max_tpl: Arc<Template>,
    /// `input_step`, above zero (`menu.py:461-462`).
    step: f64,
    /// `realtime`: run `gcode` on every settled change (`menu.py:473-481`).
    realtime: bool,
    /// The bounds as evaluated when editing began.
    min: f64,
    max: f64,
    /// The value being edited, or `None` while the item is not being edited.
    value: Option<f64>,
    /// A change is waiting for its `gcode` run (`_is_dirty`).
    dirty: bool,
    /// The heartbeat the last change happened on (`__last_change`).
    last_change: Option<f64>,
}

/// One container's children and selection (`MenuContainer` + `MenuList`,
/// `menu.py:250-566`).
#[derive(Default)]
struct ListData {
    /// Every child, enabled or not, as `(item, name as written)`
    /// (`MenuContainer._allitems`).
    allitems: Vec<(Arc<MenuItem>, String)>,
    /// The enabled children, in screen order (`MenuContainer._items`).
    items: Vec<Arc<MenuItem>>,
    /// The names of the enabled children (`MenuContainer._names`).
    names: Vec<String>,
    /// The selected row (`MenuContainer.__selected`).
    selected: Option<isize>,
    /// The first row the list shows (`MenuList._viewport_top`).
    viewport_top: usize,
    /// The containers this one hangs under, for the recursion guard
    /// (`MenuContainer._parents`).
    parents: Vec<String>,
    /// A `vsdlist` lists the SD card's files (`MenuVSDList`).
    vsd: bool,
    /// The `..` item a list inserts first (`MenuList._itemBack`).
    back: Option<Arc<MenuItem>>,
}

/// What kind of item this is (`menu_items`, `menu.py:578-584`).
enum Element {
    /// `type: disabled` — present, never enabled (`MenuDisabled`).
    Disabled,
    /// `type: command` — runs its `gcode` on a press (`MenuCommand`).
    Command,
    /// A list's own `..` item: it steps out of the menu (`MenuList.__init__`).
    Back,
    /// SD card entries a `vsdlist` lists on every populate
    /// (`MenuVSDList._populate`).
    SdFile,
    /// `type: input` — a value the up/down keys change (`MenuInput`).
    Input(InputData),
    /// `type: list` and `type: vsdlist` (`MenuList`, `MenuVSDList`).
    List(ListData),
}

/// What kind of item this is, read without holding an item's lock
/// (`Element` stripped of its state).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ElementKind {
    Disabled,
    Command,
    Back,
    SdFile,
    Input,
    List,
}

/// Everything one item holds (`MenuElement`'s attributes and its subclass
/// state).
struct ItemData {
    /// `index`: where among its parent's children the item sits (`menu.py:44`).
    index: Option<i64>,
    /// `enable` (`menu.py:45-48`, `:113-118`).
    enable: Enable,
    /// `name` (`menu.py:49-53`).
    name: Name,
    /// `gcode`, empty when the option is absent (`MenuCommand.__init__`,
    /// `menu.py:422-423`).
    gcode: Option<Arc<Template>>,
    /// The selection cursor (`menu.py:24`).
    cursor: char,
    /// The rolling state of the name.
    scroll: Scroll,
    /// The subclass state.
    element: Element,
}

impl ItemData {
    fn input(&self) -> Option<&InputData> {
        match &self.element {
            Element::Input(input) => Some(input),
            _ => None,
        }
    }

    fn input_mut(&mut self) -> Option<&mut InputData> {
        match &mut self.element {
            Element::Input(input) => Some(input),
            _ => None,
        }
    }

    fn list(&self) -> Option<&ListData> {
        match &self.element {
            Element::List(list) => Some(list),
            _ => None,
        }
    }

    fn list_mut(&mut self) -> Option<&mut ListData> {
        match &mut self.element {
            Element::List(list) => Some(list),
            _ => None,
        }
    }
}

/// One rendered screen row (`MenuList.draw_container`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MenuRow {
    /// The selection cursor (`>`; `*` while the row is being edited; otherwise
    /// a space, so the names line up).
    pub prefix: String,
    /// The item's name as the panel would show it: cut and padded to the row's
    /// width, and scrolled while it is too long.
    pub text: String,
    /// The folder marker (`>` for a container).
    pub suffix: String,
}

// ===========================================================================
// MenuItem
// ===========================================================================

/// One menu item — upstream's `MenuElement` and its subclasses.
///
/// An item is shared (`Arc`) because the registry, a container's child list and
/// the menu stack all refer to the same item. Its mutable half sits behind one
/// mutex, and no item method holds that mutex while it calls another item.
pub struct MenuItem {
    /// The item's namespace: its section's sub-name, `__main __tune`
    /// (`MenuElement._ns`).
    ns: String,
    /// The manager, weakly: the registry owns the items.
    manager: Weak<MenuManager>,
    /// The item's state.
    data: Mutex<ItemData>,
}

impl MenuItem {
    fn new(ns: String, manager: Weak<MenuManager>, data: ItemData) -> Arc<Self> {
        Arc::new(Self {
            ns,
            manager,
            data: Mutex::new(data),
        })
    }

    fn lock(&self) -> MutexGuard<'_, ItemData> {
        self.data
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The item's namespace, as the registry keys it.
    pub fn ns(&self) -> &str {
        &self.ns
    }

    /// Whether this item is a container (`isinstance(item, MenuContainer)`).
    pub fn is_container(&self) -> bool {
        matches!(self.lock().element, Element::List(_))
    }

    /// Whether this item is a `list`/`vsdlist` (`isinstance(item, MenuList)`).
    fn is_list(&self) -> bool {
        matches!(self.lock().element, Element::List(_))
    }

    /// Whether this item is an input being edited (`MenuInput.is_editing`).
    pub fn is_editing(&self) -> bool {
        self.lock()
            .input()
            .map(|input| input.value.is_some())
            .unwrap_or(false)
    }

    /// Whether a long name rolls (`is_scrollable`; false for an input,
    /// `menu.py:463-464`).
    fn is_scrollable(&self) -> bool {
        self.lock().input().is_none()
    }

    /// The value an input is showing, or `None` when it is not being edited
    /// (`MenuInput._input_value`).
    pub fn input_value(&self) -> Option<f64> {
        self.lock().input().and_then(|input| input.value)
    }

    /// The item's name as it is shown, unrolled (`_render_name`,
    /// `menu.py:81-85`).
    pub fn name_text(&self) -> String {
        let name = {
            let data = self.lock();
            match &data.name {
                Name::Literal(name) => return asflat(name),
                Name::Template(template) => Arc::clone(template),
            }
        };
        let Some(manager) = self.manager.upgrade() else {
            return String::new();
        };
        match self.render(&manager, &name, None) {
            Ok(text) => asflat(&text),
            Err(error) => {
                tracing::error!("menu '{}': name: {error}", self.ns);
                String::new()
            }
        }
    }

    /// Render `template` in this item's context (`get_context`, `menu.py:104-109`).
    ///
    /// An input item's `menu.input` is the value being edited, or — while it is
    /// not being edited — what its `input` option evaluates to
    /// (`MenuInput.get_context`, `menu.py:483-490`).
    fn render(
        &self,
        manager: &Arc<MenuManager>,
        template: &Template,
        event: Option<&str>,
    ) -> Result<String, MenuError> {
        let (editing, value_tpl) = {
            let data = self.lock();
            match data.input() {
                Some(input) => (input.value, Some(Arc::clone(&input.value_tpl))),
                None => (None, None),
            }
        };
        let input = match (editing, value_tpl) {
            (Some(value), _) => Some(value),
            (None, Some(template)) => Some(eval_template_number(manager, &self.ns, &template)?),
            (None, None) => None,
        };
        let mut context = manager.base_context();
        manager.bind_menu(&mut context, &self.ns, event, input);
        template
            .render(&mut context)
            .map_err(|error| error.to_string())
    }

    /// Render `template` without `menu.input`, the context `enable` and an
    /// input's own bounds are evaluated in (`MenuElement.get_context`).
    fn render_plain(
        &self,
        manager: &Arc<MenuManager>,
        template: &Template,
    ) -> Result<String, MenuError> {
        let mut context = manager.base_context();
        manager.bind_menu(&mut context, &self.ns, None, None);
        template
            .render(&mut context)
            .map_err(|error| error.to_string())
    }

    /// Evaluate `enable` (`eval_enable`, `menu.py:113-118`).
    ///
    /// A template that fails to render or to parse leaves the item disabled and
    /// logs: upstream lets the exception out into the screen update, which
    /// swallows it, so a broken `enable` hides the item either way.
    pub fn is_enabled(&self) -> bool {
        let enable = {
            let data = self.lock();
            match &data.enable {
                Enable::Bool(value) => return *value,
                Enable::Template(template) => Arc::clone(template),
            }
        };
        let Some(manager) = self.manager.upgrade() else {
            return false;
        };
        match self
            .render_plain(&manager, &enable)
            .and_then(|text| literal_truth(&text))
        {
            Ok(value) => value,
            Err(error) => {
                tracing::error!("menu '{}': enable: {error}", self.ns);
                false
            }
        }
    }

    /// The name as shown, rolled when selected (`render_name`,
    /// `menu.py:148-165`).
    fn render_name_selected(&self, selected: bool) -> String {
        let name = self.name_text();
        let pos = {
            let mut data = self.lock();
            if selected && data.scroll.pos.is_some() {
                data.scroll.pos
            } else {
                data.scroll.pos = None;
                data.scroll.request = Some(false);
                None
            }
        };
        match pos {
            Some(index) => slice_name(&name, index),
            None => name,
        }
    }

    /// Tell the scroller what to do next (`need_scroller`, `menu.py:130-137`).
    fn need_scroller(&self, request: Option<bool>) {
        self.lock().scroll.request = request;
    }

    /// One heartbeat: remember it, step the scroller and run a settled input
    /// change (`menu.py:119-128`, `:473-481`).
    fn heartbeat(&self, eventtime: f64) {
        let settled_change = {
            let mut data = self.lock();
            data.scroll.last_heartbeat = Some(eventtime);
            if eventtime >= data.scroll.next {
                data.scroll.next = eventtime + SCROLL_DELAY;
                let editing = data
                    .input()
                    .map(|input| input.value.is_some())
                    .unwrap_or(false);
                if !editing {
                    match (data.scroll.pos, data.scroll.request) {
                        (None, Some(true)) => data.scroll.pos = Some(0),
                        (_, Some(true)) => {
                            let pos = data.scroll.pos.unwrap_or(0);
                            data.scroll.pos = Some(pos + 1);
                            data.scroll.request = Some(false);
                        }
                        (_, Some(false)) => {}
                        (_, None) => {
                            data.scroll.pos = None;
                            data.scroll.request = Some(false);
                        }
                    }
                }
            }
            let Some(input) = data.input_mut() else {
                return;
            };
            let waiting = input.dirty
                && input.value.is_some()
                && input
                    .last_change
                    .map(|last| eventtime - last > INPUT_CHANGE_DELAY)
                    .unwrap_or(false);
            if !waiting {
                return;
            }
            input.dirty = false;
            input.realtime
        };
        if settled_change {
            self.run_gcode(Some("change"));
        }
    }

    /// Called when the item becomes the selected row (`select`, `menu.py:120`).
    fn select(&self) {
        let mut data = self.lock();
        data.scroll.pos = None;
        data.scroll.request = Some(false);
    }

    /// Render and queue the item's `gcode` (`run_script`, `menu.py:198-217`).
    fn run_gcode(&self, event: Option<&str>) -> String {
        let gcode = {
            let data = self.lock();
            match &data.gcode {
                Some(template) => Arc::clone(template),
                None => return String::new(),
            }
        };
        let Some(manager) = self.manager.upgrade() else {
            return String::new();
        };
        let script = match self.render(&manager, &gcode, event) {
            Ok(script) => script,
            Err(error) => {
                tracing::error!("menu '{}': gcode: {error}", self.ns);
                return String::new();
            }
        };
        manager.queue_gcode(&script);
        script
    }

    /// What a press does on a non-container item (`MenuManager.press`,
    /// `menu.py:919-942`).
    fn press(&self, event: &str) {
        // The kind is read and the lock dropped before anything acts: an
        // action below locks this item again.
        let kind = self.kind();
        match kind {
            ElementKind::Input => {
                if self.is_editing() {
                    self.run_gcode(Some(event));
                }
                // `handle_script_click` (`menu.py:514-518`).
                if self.is_editing() {
                    self.stop_editing();
                } else {
                    self.start_editing();
                }
            }
            ElementKind::Command | ElementKind::SdFile => {
                self.run_gcode(Some(event));
            }
            ElementKind::Back => {
                if let Some(manager) = self.manager.upgrade() {
                    manager.back(false, true);
                }
            }
            ElementKind::Disabled | ElementKind::List => {}
        }
    }

    /// Which kind of item this is, without holding the lock.
    fn kind(&self) -> ElementKind {
        match self.lock().element {
            Element::Disabled => ElementKind::Disabled,
            Element::Command => ElementKind::Command,
            Element::Back => ElementKind::Back,
            Element::SdFile => ElementKind::SdFile,
            Element::Input(_) => ElementKind::Input,
            Element::List(_) => ElementKind::List,
        }
    }

    /// Start editing an input (`MenuInput.start_editing`, `menu.py:466-472`).
    pub fn start_editing(&self) {
        let Some(manager) = self.manager.upgrade() else {
            return;
        };
        let (value_tpl, min_tpl, max_tpl) = {
            let data = self.lock();
            let Some(input) = data.input() else {
                return;
            };
            if input.value.is_some() {
                return;
            }
            (
                Arc::clone(&input.value_tpl),
                Arc::clone(&input.min_tpl),
                Arc::clone(&input.max_tpl),
            )
        };
        let min = self.eval_number(&manager, &min_tpl, "input_min");
        let max = self.eval_number(&manager, &max_tpl, "input_max");
        let value = self.eval_number(&manager, &value_tpl, "input");
        let mut data = self.lock();
        let last_heartbeat = data.scroll.last_heartbeat;
        let Some(input) = data.input_mut() else {
            return;
        };
        input.min = min;
        input.max = max;
        input.value = Some(max.min(value.max(min)));
        input.last_change = last_heartbeat;
        input.dirty = true;
    }

    /// Stop editing, dropping the value (`MenuInput.stop_editing`,
    /// `menu.py:455-459`).
    pub fn stop_editing(&self) {
        let mut data = self.lock();
        if let Some(input) = data.input_mut() {
            input.value = None;
        }
    }

    fn eval_number(&self, manager: &Arc<MenuManager>, template: &Template, what: &str) -> f64 {
        let text = match self.render_plain(manager, template) {
            Ok(text) => text,
            Err(error) => {
                tracing::error!("menu '{}': {what}: {error}", self.ns);
                return 0.0;
            }
        };
        match literal_number(&text) {
            Ok(value) => value,
            Err(error) => {
                tracing::error!("menu '{}': {what}: {error}", self.ns);
                0.0
            }
        }
    }

    /// One up/down step, with the `fast_rate` acceleration
    /// (`_get_input_step`, `menu.py:498-502`).
    fn input_step(&self, fast_rate: bool) -> f64 {
        let data = self.lock();
        let Some(input) = data.input() else {
            return 0.0;
        };
        let span = (input.max - input.min) / input.step;
        if fast_rate && span > 100.0 {
            10.0 * input.step
        } else {
            input.step
        }
    }

    fn change_value(&self, delta: f64) {
        let mut data = self.lock();
        let last_heartbeat = data.scroll.last_heartbeat;
        let Some(input) = data.input_mut() else {
            return;
        };
        let Some(value) = input.value else { return };
        let next = (value + delta).max(input.min).min(input.max);
        if next != value {
            input.value = Some(next);
            input.last_change = last_heartbeat;
            input.dirty = true;
        }
    }

    /// Add one step, clamped (`MenuInput.inc_value`, `menu.py:504-518`).
    pub fn inc_value(&self, fast_rate: bool) {
        self.change_value(self.input_step(fast_rate).abs());
    }

    /// Subtract one step, clamped (`MenuInput.dec_value`, `menu.py:520-534`).
    pub fn dec_value(&self, fast_rate: bool) {
        self.change_value(-self.input_step(fast_rate).abs());
    }

    // -- container half -----------------------------------------------------

    /// Rebuild the child list (`MenuContainer.populate`, `menu.py:319-334`).
    fn populate(&self) {
        let Some(manager) = self.manager.upgrade() else {
            return;
        };
        let ns = self.ns.clone();
        let (vsd, back) = {
            let mut data = self.lock();
            let Some(list) = data.list_mut() else { return };
            list.allitems.clear();
            list.viewport_top = 0;
            (list.vsd, list.back.clone())
        };
        let mut allitems: Vec<(Arc<MenuItem>, String)> = Vec::new();
        for name in manager.lookup_children(&ns) {
            let target = resolve_ns(&ns, &name);
            let Some(item) = manager.lookup_menuitem(&target) else {
                continue;
            };
            self.adopt_child(&item);
            allitems.push((item, name));
        }
        if vsd {
            for (fname, _size) in sd_files(&manager) {
                let item = manager.command_item(&format!("M23 /{fname}"), &format!("'{fname}'"));
                allitems.push((item, format!("'{fname}'")));
            }
        }
        if let Some(back) = back {
            allitems.insert(0, (back, "..".to_string()));
        }
        let mut data = self.lock();
        if let Some(list) = data.list_mut() {
            list.allitems = allitems;
        }
    }

    /// Record that a container child hangs under this one (`_insert_item`'s
    /// `add_parents` + `assert_recursive_relation`, `menu.py:271-291`).
    fn adopt_child(&self, child: &Arc<MenuItem>) {
        if !child.is_container() {
            return;
        }
        let parents = {
            let data = self.lock();
            let mut parents = data.list().map(|l| l.parents.clone()).unwrap_or_default();
            parents.push(self.ns.clone());
            parents
        };
        let recursive = {
            let mut data = child.lock();
            match data.list_mut() {
                Some(list) => {
                    let recursive = parents.contains(&child.ns);
                    list.parents.extend(parents);
                    recursive
                }
                None => false,
            }
        };
        if recursive {
            tracing::error!("menu: recursive relation of '{}' container", child.ns);
        }
    }

    /// List the container's enabled children (`update_items`, `menu.py:336-341`).
    fn update_items(&self) {
        let allitems = {
            let data = self.lock();
            data.list()
                .map(|list| list.allitems.clone())
                .unwrap_or_default()
        };
        let enabled: Vec<(Arc<MenuItem>, String)> = allitems
            .into_iter()
            .filter(|(item, _)| item.is_enabled())
            .collect();
        let mut data = self.lock();
        let Some(list) = data.list_mut() else { return };
        list.items = enabled.iter().map(|(item, _)| Arc::clone(item)).collect();
        list.names = enabled.iter().map(|(_, name)| name.clone()).collect();
    }

    /// The container's enabled child count (`__len__`).
    pub fn len(&self) -> usize {
        self.lock().list().map(|list| list.items.len()).unwrap_or(0)
    }

    /// Whether the container lists nothing.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The selected row index (`MenuContainer.selected`).
    fn selected_index(&self) -> Option<isize> {
        self.lock().list().and_then(|list| list.selected)
    }

    /// The selected item (`selected_item`, `menu.py:370-374`).
    pub fn selected_item(&self) -> Option<Arc<MenuItem>> {
        let data = self.lock();
        let list = data.list()?;
        let index = list.selected?;
        if index < 0 {
            return None;
        }
        list.items.get(index as usize).cloned()
    }

    /// Select row `index` (`select_at`, `menu.py:355-360`).
    fn select_at(&self, index: Option<isize>) {
        {
            let mut data = self.lock();
            let Some(list) = data.list_mut() else { return };
            list.selected = index;
        }
        if let Some(item) = self.selected_item() {
            item.select();
        }
    }

    /// Select the first row (`init_selection`, `menu.py:351-352`).
    fn init_selection(&self) {
        self.select_at(Some(0));
    }

    /// `select_next` (`menu.py:376-384`).
    fn select_next(&self) {
        let selected = self.selected_index();
        let len = self.len() as isize;
        let index = match selected {
            None => Some(0),
            Some(selected) if selected >= 0 && selected < len - 1 => Some(selected + 1),
            Some(selected) => Some(selected),
        };
        self.select_at(index);
    }

    /// `select_prev` (`menu.py:386-394`).
    fn select_prev(&self) {
        let selected = self.selected_index();
        let len = self.len() as isize;
        let index = match selected {
            None => Some(0),
            Some(selected) if selected > 0 && selected < len => Some(selected - 1),
            Some(selected) => Some(selected),
        };
        self.select_at(index);
    }

    /// Where `item` sits among this container's rows (`index_of`,
    /// `menu.py:297-306`).
    fn index_of(&self, item: &Arc<MenuItem>, look_inside: bool) -> Option<isize> {
        if let Some(index) = self.index_of_direct(item) {
            return Some(index);
        }
        if !look_inside {
            return None;
        }
        let children = {
            let data = self.lock();
            data.list()
                .map(|list| list.items.clone())
                .unwrap_or_default()
        };
        for child in children {
            if child.is_container() && child.index_of_direct(item).is_some() {
                if let Some(index) = self.index_of_direct(&child) {
                    return Some(index);
                }
            }
        }
        None
    }

    fn index_of_direct(&self, item: &Arc<MenuItem>) -> Option<isize> {
        let data = self.lock();
        let list = data.list()?;
        list.items
            .iter()
            .position(|child| Arc::ptr_eq(child, item))
            .map(|index| index as isize)
    }

    /// Lay the container's visible rows out (`draw_container`, `menu.py:499-566`).
    fn draw_rows(&self, manager: &MenuManager, eventtime: f64) -> Vec<MenuRow> {
        let nrows = manager.rows();
        let cols = manager.cols();
        let selected_row = self.selected_index();
        let len = self.len();
        let (top, items) = {
            let mut data = self.lock();
            let Some(list) = data.list_mut() else {
                return Vec::new();
            };
            // Adjust the viewport around the selection, then clamp it.
            match selected_row {
                Some(selected) if selected >= 0 => {
                    let selected = selected as usize;
                    if selected >= list.viewport_top + nrows {
                        list.viewport_top = (selected - nrows) + 1;
                    }
                    if selected < list.viewport_top {
                        list.viewport_top = selected;
                    }
                }
                _ => list.viewport_top = 0,
            }
            list.viewport_top = list.viewport_top.min(len.saturating_sub(nrows));
            (list.viewport_top, list.items.clone())
        };
        let mut rows = Vec::new();
        for row in top..top + nrows {
            let Some(current) = items.get(row) else {
                rows.push(MenuRow {
                    prefix: String::new(),
                    text: " ".repeat(cols),
                    suffix: String::new(),
                });
                continue;
            };
            let selected = selected_row == Some(row as isize);
            if selected {
                current.heartbeat(eventtime);
            }
            let text = current.render_name_selected(selected);
            let editing = current.is_editing();
            let prefix = if selected && !editing {
                current.lock().cursor.to_string()
            } else if selected {
                "*".to_string()
            } else {
                " ".to_string()
            };
            let suffix = if current.is_list() {
                ">".to_string()
            } else {
                String::new()
            };
            let prefix_len = prefix.chars().count();
            let suffix_len = suffix.chars().count();
            let width = cols.saturating_sub(prefix_len + suffix_len);
            let text_len = text.chars().count();
            // `draw_text` answers the column after the text, so the scroller
            // starts when that column is past the panel (`menu.py:548-555`).
            let text_column = prefix_len + text_len.max(width);
            let scroller = selected && text_column > cols && current.is_scrollable();
            current.need_scroller(if scroller { Some(true) } else { None });
            rows.push(MenuRow {
                prefix,
                text: pad_to(&truncate_to(&text, width), width),
                suffix,
            });
        }
        rows
    }
}

/// The SD card's files, or nothing when the machine has no `virtual_sdcard`
/// (`MenuVSDList._populate`, `menu.py:568-576`).
fn sd_files(manager: &Arc<MenuManager>) -> Vec<(String, u64)> {
    let Some(sdcard) = manager
        .printer()
        .lookup_object_as::<VirtualSdCard>(VIRTUAL_SDCARD_OBJECT)
    else {
        return Vec::new();
    };
    match sdcard.get_file_list(false) {
        Ok(files) => files,
        Err(error) => {
            tracing::error!("menu: {error}");
            Vec::new()
        }
    }
}

// ===========================================================================
// MenuManager
// ===========================================================================

/// The `menu` printer object: the tree, the stack of open containers, the
/// queued scripts and the key events (upstream `MenuManager`, `menu.py:586`).
pub struct MenuManager {
    /// The machine.
    printer: Arc<Printer>,
    /// The display this menu draws on, for a redraw request.
    display: Weak<PrinterLCD>,
    /// The reactor the timers and the script queue run on.
    reactor: Arc<dyn Reactor>,
    /// `gcode_macro`, which compiles every scripted option.
    gcode_macro: Arc<PrinterGCodeMacro>,
    /// The panel's size in characters (`display.get_dimensions()`).
    cols: usize,
    rows: usize,
    /// `menu_timeout`: seconds of inactivity before the menu closes; 0 = never.
    timeout: i64,
    /// `menu_reverse_navigation`.
    reverse_navigation: bool,
    /// `menu_root`: the container the menu opens at.
    root_ns: String,
    /// The items and their parent/child relation.
    registry: Mutex<Registry>,
    /// The open containers and the runtime flags.
    state: Mutex<State>,
    /// The menu's own timer (the timeout counter).
    timer: Mutex<Option<TimerHandle>>,
    /// The armed `long_click` timer.
    click_timer: Mutex<Option<TimerHandle>>,
    /// Whether the press in progress still counts as a short click.
    is_short_click: Mutex<bool>,
    /// A handle to ourselves, for the key callbacks and the queue.
    self_ref: Weak<MenuManager>,
}

/// Every item by namespace, and each container's children
/// (`MenuManager.menuitems` / `.children`).
#[derive(Default)]
struct Registry {
    items: BTreeMap<String, Arc<MenuItem>>,
    children: BTreeMap<String, Vec<String>>,
}

/// The manager's runtime state (`running` / `menustack` / …).
#[derive(Default)]
struct State {
    /// Whether the menu is open (`running`).
    running: bool,
    /// The open containers, innermost last (`menustack`).
    stack: Vec<Arc<MenuItem>>,
    /// Seconds the menu has been idle (`timer`).
    timer: i64,
    /// Rendered scripts waiting to run (`gcode_queue`).
    queue: Vec<String>,
}

impl MenuManager {
    /// Build the menu for `display`: load the shipped layout, overlay the main
    /// config's `[menu ...]` sections, resolve the root, register the `menu`
    /// object and the key buttons (`menu.py:689-722`).
    ///
    /// # Errors
    /// A `[menu ...]` section with an unknown `type` or a missing `name`, an
    /// unparsable template, an unknown `menu_root`, or a duplicate `menu`
    /// registration.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        options: &MenuOptions,
        display: &Arc<PrinterLCD>,
    ) -> Result<Arc<Self>, ConfigError> {
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let root_ns = options.root.trim().to_string();
        let (cols, rows) = display.get_dimensions();
        let manager = Arc::new_cyclic(|weak| Self {
            printer: Arc::clone(printer),
            display: Arc::downgrade(display),
            reactor: printer.reactor(),
            gcode_macro,
            cols,
            rows,
            timeout: options.timeout,
            reverse_navigation: options.reverse_navigation,
            root_ns,
            registry: Mutex::new(Registry::default()),
            state: Mutex::new(State::default()),
            timer: Mutex::new(None),
            click_timer: Mutex::new(None),
            is_short_click: Mutex::new(false),
            self_ref: weak.clone(),
        });

        // The shipped layout first, then the main config's items, which replace
        // a shipped item of the same name (`menu.py:713-720`).
        let published = published_config()?;
        for section in published.get_sections_by_id("menu") {
            manager.add_section(&ConfigWrapper::untracked(section))?;
        }
        for section in config.sibling_prefix_sections("menu") {
            manager.add_section(&section)?;
        }
        if manager.lookup_menuitem(&manager.root_ns).is_none() {
            return Err(ConfigError::new(format!(
                "Unknown menuitem '{}'",
                manager.root_ns
            )));
        }
        manager.register_keys(config, options)?;

        printer.add_object(MENU_OBJECT, Arc::clone(&manager) as Arc<dyn PrinterObject>)?;

        let ready = Arc::downgrade(&manager);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(manager) = ready.upgrade() {
                    manager.start_timer();
                }
            }),
        );
        Ok(manager)
    }

    /// The machine this menu belongs to.
    pub fn printer(&self) -> &Arc<Printer> {
        &self.printer
    }

    /// The panel's width in characters.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// The panel's height in characters.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// `menu_timeout`, as configured.
    pub fn timeout(&self) -> i64 {
        self.timeout
    }

    fn registry(&self) -> MutexGuard<'_, Registry> {
        self.registry
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn state(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The globals every menu script sees: the macro context without `params`
    /// (`create_template_context`, `gcode_macro.py:106-113`).
    fn base_context(&self) -> Context {
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

    /// The `menu` value a script sees: `eventtime`, `ns`, `event`, `input`
    /// (`update_context` + `get_context`, `menu.py:781-796`).
    fn bind_menu(&self, context: &mut Context, ns: &str, event: Option<&str>, input: Option<f64>) {
        let mut menu = Map::new();
        menu.insert("eventtime".to_string(), json!(self.printer.eventtime()));
        menu.insert("ns".to_string(), json!(ns));
        if let Some(event) = event {
            menu.insert("event".to_string(), json!(event));
        }
        if let Some(input) = input {
            menu.insert("input".to_string(), json!(input));
        }
        context.insert("menu", Rt::Json(Value::Object(menu)));
    }

    // -- loading ------------------------------------------------------------

    /// Load one `[menu <name>]` section (`load_menuitems`, `menu.py:1035-1042`).
    fn add_section(&self, config: &ConfigWrapper) -> Result<(), ConfigError> {
        let kind = config.get_choice("type", MENU_ITEM_TYPES, None)?;
        let item = self.build_item(config, kind.as_str())?;
        self.add_menuitem(item.ns().to_string(), item);
        Ok(())
    }

    /// Build one item from its section (`MenuElement.__init__` +
    /// `menu_items[type](self, cfg)`).
    fn build_item(&self, config: &ConfigWrapper, kind: &str) -> Result<Arc<MenuItem>, ConfigError> {
        let ns = config
            .section()
            .sub
            .clone()
            .unwrap_or_default()
            .trim()
            .to_string();
        let index = config.get_optional_int("index")?;
        let enable = load_enable(config)?;
        let name = load_name(config, &self.gcode_macro)?;
        let element = match kind {
            "disabled" => Element::Disabled,
            "command" => Element::Command,
            "input" => Element::Input(load_input(config, &self.gcode_macro)?),
            "list" => Element::List(ListData::default()),
            _ => Element::List(ListData {
                vsd: true,
                ..ListData::default()
            }),
        };
        let gcode = match element {
            Element::Command | Element::Input(_) => Some(Arc::new(
                self.gcode_macro.load_template(config, "gcode", Some(""))?,
            )),
            _ => None,
        };
        let data = ItemData {
            index,
            enable,
            name,
            gcode,
            cursor: '>',
            scroll: Scroll::default(),
            element,
        };
        let item = MenuItem::new(ns.clone(), self.self_ref.clone(), data);
        // A list's `..` item is its own (`MenuList.__init__`, `menu.py:428-436`).
        if item.is_list() {
            let back = MenuItem::new(
                format!("menu {ns} .."),
                self.self_ref.clone(),
                ItemData {
                    index: None,
                    enable: Enable::Bool(true),
                    name: Name::Literal("..".to_string()),
                    gcode: None,
                    cursor: '>',
                    scroll: Scroll::default(),
                    element: Element::Back,
                },
            );
            let mut data = item.lock();
            if let Some(list) = data.list_mut() {
                list.back = Some(back);
            }
        }
        Ok(item)
    }

    /// An item whose `gcode` is a fixed string (the SD-card entries,
    /// `MenuVSDList._populate`, `menu.py:568-576`).
    fn command_item(&self, gcode: &str, name: &str) -> Arc<MenuItem> {
        let template = Template::parse(&format!("menu vsdlist:gcode {name}"), gcode)
            .expect("an `M23 /<file>` script parses");
        MenuItem::new(
            format!("menu vsdlist {name}"),
            self.self_ref.clone(),
            ItemData {
                index: None,
                enable: Enable::Bool(true),
                name: Name::Literal(name.to_string()),
                gcode: Some(Arc::new(template)),
                cursor: '>',
                scroll: Scroll::default(),
                element: Element::SdFile,
            },
        )
    }

    /// Register one item by namespace, and under its parent (`add_menuitem`,
    /// `menu.py:1004-1020`).
    fn add_menuitem(&self, name: String, item: Arc<MenuItem>) {
        let mut registry = self.registry();
        let existing = registry.items.contains_key(&name);
        if existing {
            tracing::info!("menu: declaration of '{name}' hides the previous declaration");
        }
        registry.items.insert(name.clone(), Arc::clone(&item));
        let parent = parent_ns(&name);
        if parent.is_empty() || existing {
            return;
        }
        let index = item.lock().index;
        let children = registry.children.entry(parent).or_default();
        match index {
            Some(index) => {
                let index = (index.max(0) as usize).min(children.len());
                children.insert(index, name);
            }
            None => children.push(name),
        }
    }

    /// Look one item up by namespace (`lookup_menuitem`, `menu.py:1022-1028`).
    pub fn lookup_menuitem(&self, name: &str) -> Option<Arc<MenuItem>> {
        self.registry().items.get(name).cloned()
    }

    /// The children of a container, in insertion order (`lookup_children`,
    /// `menu.py:1029-1032`).
    fn lookup_children(&self, ns: &str) -> Vec<String> {
        self.registry()
            .children
            .get(ns)
            .cloned()
            .unwrap_or_default()
    }

    /// How many items the tree holds (tests and diagnostics).
    pub fn item_count(&self) -> usize {
        self.registry().items.len()
    }

    // -- keys ---------------------------------------------------------------

    /// Register the key buttons (`MenuKeys.__init__`, `menu_keys.py:13-43`).
    fn register_keys(
        &self,
        config: &ConfigWrapper,
        options: &MenuOptions,
    ) -> Result<(), ConfigError> {
        let keys = &options.keys;
        if keys.encoder_pins.is_some() {
            tracing::warn!(
                "menu: 'encoder_pins' is configured but this host's button registry has no \
                 rotary encoder, so the encoder keys never reach the menu"
            );
        }
        let buttons = PrinterButtons::ensure(&self.printer)?;
        for (option, event) in KEY_PINS {
            let Some(pin) = keys.pin(option) else {
                continue;
            };
            let analog = format!("analog_range_{option}");
            if config.has(&analog) {
                tracing::warn!(
                    "menu: '{analog}' is configured; analog buttons are not implemented in this \
                     host, so '{option}' is not registered"
                );
                continue;
            }
            let weak = self.self_ref.clone();
            let event = event.to_string();
            let is_click = event == "click";
            buttons.register_buttons(
                vec![pin],
                Box::new(move |eventtime, state| {
                    let Some(manager) = weak.upgrade() else {
                        return;
                    };
                    if is_click {
                        manager.click_callback(eventtime, state);
                    } else if state {
                        manager.key_event(&event, eventtime);
                    }
                }),
            );
        }
        if let Some(pin) = &keys.kill_pin {
            let printer = Arc::downgrade(&self.printer);
            buttons.register_buttons(
                vec![pin.clone()],
                Box::new(move |_, state| {
                    if state {
                        if let Some(printer) = printer.upgrade() {
                            printer.invoke_shutdown("Shutdown due to kill button!");
                        }
                    }
                }),
            );
        }
        Ok(())
    }

    /// A press or release of the click button (`click_callback`,
    /// `menu_keys.py:77-84`).
    fn click_callback(&self, eventtime: f64, state: bool) {
        if state {
            *self
                .is_short_click
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()) = true;
            let weak = self.self_ref.clone();
            let handle = self.reactor.call_later(
                LONG_PRESS_DURATION,
                Box::new(move |eventtime| {
                    if let Some(manager) = weak.upgrade() {
                        manager.long_click_event(eventtime);
                    }
                }),
            );
            let previous = self
                .click_timer
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .replace(handle);
            if let Some(previous) = previous {
                previous.cancel();
            }
            return;
        }
        let short = {
            let mut armed = self
                .is_short_click
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            let short = *armed;
            *armed = false;
            short
        };
        if !short {
            return;
        }
        if let Some(handle) = self
            .click_timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take()
        {
            handle.cancel();
        }
        self.key_event("click", eventtime);
    }

    /// The long-press timer fired (`long_click_event`, `menu_keys.py:71-74`).
    fn long_click_event(&self, eventtime: f64) {
        *self
            .is_short_click
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = false;
        self.key_event("long_click", eventtime);
    }

    /// Map one key to its action (`key_event`, `menu.py:1052-1068`).
    pub fn key_event(&self, key: &str, eventtime: f64) {
        match key {
            "click" | "long_click" => self.click(key, eventtime),
            "up" => self.up(false),
            "fast_up" => self.up(true),
            "down" => self.down(false),
            "fast_down" => self.down(true),
            "back" => self.back(false, true),
            _ => {}
        }
        if let Some(display) = self.display.upgrade() {
            display.request_redraw();
        }
    }

    /// A click opens the menu, or presses the selected item
    /// (`_click_callback`, `menu.py:1044-1050`).
    fn click(&self, event: &str, eventtime: f64) {
        if self.is_running() {
            self.press(event);
        } else {
            self.begin(eventtime);
        }
    }

    // -- the timer ----------------------------------------------------------

    /// `handle_ready` (`menu.py:722-724`): start the timeout timer.
    fn start_timer(&self) {
        let weak = self.self_ref.clone();
        let handle = self.reactor.register_timer_named(
            "menu",
            Box::new(move |eventtime| {
                let manager = weak.upgrade()?;
                manager.timeout_check(eventtime);
                Some(eventtime + TIMER_DELAY)
            }),
            self.reactor.monotonic(),
        );
        *self
            .timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(handle);
    }

    /// Count the idle seconds and close the menu when `menu_timeout` runs out
    /// (`timeout_check`, `menu.py:726-736`).
    fn timeout_check(&self, _eventtime: f64) {
        let running = self.is_running();
        let root_is_container = self
            .lookup_menuitem(&self.root_ns)
            .map(|root| root.is_container())
            .unwrap_or(false);
        let expired = {
            let mut state = self.state();
            if running && self.timeout > 0 && root_is_container {
                if state.timer >= self.timeout {
                    true
                } else {
                    state.timer += 1;
                    false
                }
            } else {
                state.timer = 0;
                false
            }
        };
        if expired {
            self.exit(false);
        }
    }

    // -- the stack ----------------------------------------------------------

    /// Whether the menu is open (`is_running`).
    pub fn is_running(&self) -> bool {
        self.state().running
    }

    /// Open the menu at the root (`begin`, `menu.py:738-752`).
    pub fn begin(&self, _eventtime: f64) {
        {
            let mut state = self.state();
            state.stack.clear();
            state.timer = 0;
        }
        let Some(root) = self.lookup_menuitem(&self.root_ns) else {
            tracing::error!("menu: invalid root '{}', menu stopped", self.root_ns);
            return;
        };
        if !root.is_container() {
            tracing::error!("menu: invalid root '{}', menu stopped", self.root_ns);
            return;
        }
        root.init_selection();
        self.stack_push(root);
        self.state().running = true;
    }

    /// Push a container: populate it, run the `enter`/`leave` scripts, select
    /// its first row (`stack_push`, `menu.py:798-810`).
    fn stack_push(&self, container: Arc<MenuItem>) {
        container.populate();
        if let Some(top) = self.stack_peek(0) {
            if top.is_list() {
                top.run_gcode(Some("leave"));
            }
        }
        if container.is_list() {
            container.run_gcode(Some("enter"));
        }
        if !container.is_editing() {
            container.update_items();
            container.init_selection();
        }
        self.state().stack.push(container);
    }

    /// Pop the innermost container, reselecting where we came from
    /// (`stack_pop`, `menu.py:812-833`).
    fn stack_pop(&self, update: bool) -> Option<Arc<MenuItem>> {
        let container = self.state().stack.pop()?;
        let Some(top) = self.stack_peek(0) else {
            if container.is_list() {
                container.run_gcode(Some("leave"));
            }
            return Some(container);
        };
        if !top.is_editing() && update {
            top.update_items();
            top.init_selection();
        }
        if container.is_list() {
            container.run_gcode(Some("leave"));
        }
        if top.is_list() {
            top.run_gcode(Some("enter"));
        }
        Some(container)
    }

    /// How many containers are open (`stack_size`).
    pub fn stack_size(&self) -> usize {
        self.state().stack.len()
    }

    /// The container `lvl` levels from the top (`stack_peek`).
    pub fn stack_peek(&self, lvl: usize) -> Option<Arc<MenuItem>> {
        let state = self.state();
        let size = state.stack.len();
        if size > lvl {
            return state.stack.get(size - lvl - 1).cloned();
        }
        None
    }

    /// The open containers' namespaces, outermost first (tests and
    /// diagnostics).
    pub fn stack_namespace(&self) -> Vec<String> {
        self.state()
            .stack
            .iter()
            .map(|item| item.ns().to_string())
            .collect()
    }

    /// The selected row of the innermost container, or `None` (tests and
    /// diagnostics).
    pub fn selected_namespace(&self) -> Option<String> {
        self.stack_peek(0)?
            .selected_item()
            .map(|item| item.ns().to_string())
    }

    // -- the screen ---------------------------------------------------------

    /// The refresh hook (`screen_update_event`, `menu.py:835-846`): `true`
    /// while the menu owns the screen.
    pub fn screen_update_event(&self, eventtime: f64) -> bool {
        if !self.is_running() {
            return false;
        }
        if let Some(container) = self.stack_peek(0) {
            container.heartbeat(eventtime);
        }
        true
    }

    /// The rows the panel would show now (`MenuList.draw_container`).
    ///
    /// The framework has no `draw_text`, so the rows are computed and handed
    /// back instead of painted; they carry the cursor, the folder marker and the
    /// scrolled name exactly as a drawing would.
    pub fn screen_rows(&self) -> Vec<MenuRow> {
        let eventtime = self.printer.eventtime();
        self.screen_rows_at(eventtime)
    }

    fn screen_rows_at(&self, eventtime: f64) -> Vec<MenuRow> {
        match self.stack_peek(0) {
            Some(container) => container.draw_rows(self, eventtime),
            None => Vec::new(),
        }
    }

    // -- navigation ---------------------------------------------------------

    /// `up` (`menu.py:848-860`), or a `dec_value` on an input being edited.
    pub fn up(&self, fast_rate: bool) {
        let Some(container) = self.stack_peek(0) else {
            return;
        };
        if !self.is_running() {
            return;
        }
        self.state().timer = 0;
        if let Some(current) = container.selected_item() {
            if current.is_editing() {
                current.dec_value(fast_rate);
                return;
            }
        }
        if self.reverse_navigation {
            container.select_next();
        } else {
            container.select_prev();
        }
    }

    /// `down` (`menu.py:861-873`), or an `inc_value` on an input being edited.
    pub fn down(&self, fast_rate: bool) {
        let Some(container) = self.stack_peek(0) else {
            return;
        };
        if !self.is_running() {
            return;
        }
        self.state().timer = 0;
        if let Some(current) = container.selected_item() {
            if current.is_editing() {
                current.inc_value(fast_rate);
                return;
            }
        }
        if self.reverse_navigation {
            container.select_prev();
        } else {
            container.select_next();
        }
    }

    /// Step out of the innermost container (`back`, `menu.py:874-897`).
    pub fn back(&self, force: bool, update: bool) {
        let Some(container) = self.stack_peek(0) else {
            return;
        };
        if !self.is_running() {
            return;
        }
        self.state().timer = 0;
        if let Some(current) = container.selected_item() {
            if current.is_editing() {
                if !force {
                    return;
                }
                current.stop_editing();
            }
        }
        match self.stack_peek(1) {
            Some(parent) => {
                self.stack_pop(update);
                match parent.index_of(&container, true) {
                    Some(index) => parent.select_at(Some(index)),
                    None => {
                        if parent.selected_item().is_none() {
                            parent.init_selection();
                        }
                    }
                }
            }
            None => {
                self.stack_pop(update);
                self.state().running = false;
            }
        }
    }

    /// Close the menu (`exit`, `menu.py:898-909`).
    pub fn exit(&self, force: bool) {
        let Some(container) = self.stack_peek(0) else {
            return;
        };
        if !self.is_running() {
            return;
        }
        self.state().timer = 0;
        if !force {
            if let Some(current) = container.selected_item() {
                if current.is_editing() {
                    return;
                }
            }
        }
        if container.is_list() {
            container.run_gcode(Some("leave"));
        }
        self.state().running = false;
    }

    /// Open a container if it is one and nothing is being edited
    /// (`push_container`, `menu.py:910-917`).
    pub fn push_container(&self, menu: &Arc<MenuItem>) -> bool {
        let Some(container) = self.stack_peek(0) else {
            return false;
        };
        if self.is_running()
            && container.is_container()
            && menu.is_container()
            && !container.is_editing()
            && !Arc::ptr_eq(menu, &container)
        {
            self.stack_push(Arc::clone(menu));
            return true;
        }
        false
    }

    /// Press the selected item (`press`, `menu.py:919-942`).
    pub fn press(&self, event: &str) {
        let Some(container) = self.stack_peek(0) else {
            return;
        };
        if !self.is_running() {
            return;
        }
        self.state().timer = 0;
        let Some(current) = container.selected_item() else {
            // No selection: pass the press to the container itself.
            container.run_gcode(Some(event));
            return;
        };
        if current.is_container() {
            self.stack_push(current);
        } else {
            current.press(event);
        }
    }

    // -- scripts ------------------------------------------------------------

    /// Queue a rendered script (`queue_gcode`, `menu.py:943-949`).
    ///
    /// The first script scheduled arms one reactor callback that drains the
    /// queue, so the scripts of one key press run in order and none runs inside
    /// the refresh that rendered it.
    pub fn queue_gcode(&self, script: &str) {
        if script.is_empty() {
            return;
        }
        let schedule = {
            let mut state = self.state();
            let schedule = state.queue.is_empty();
            state.queue.push(script.to_string());
            schedule
        };
        if !schedule {
            return;
        }
        let weak = self.self_ref.clone();
        self.reactor.call_later(
            0.0,
            Box::new(move |_| {
                if let Some(manager) = weak.upgrade() {
                    manager.dispatch_gcode();
                }
            }),
        );
    }

    /// The scripts waiting to run, in order (tests and diagnostics).
    pub fn queued_gcode(&self) -> Vec<String> {
        self.state().queue.clone()
    }

    /// Run every queued script (`dispatch_gcode`, `menu.py:955-963`).
    ///
    /// Called from the reactor, so the scripts run detached (`tokio::spawn`),
    /// the way a button's own script does.
    pub fn dispatch_gcode(&self) {
        let scripts = {
            let mut state = self.state();
            std::mem::take(&mut state.queue)
        };
        if scripts.is_empty() {
            return;
        }
        let Some(gcode) = self.printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
            tracing::error!("menu: no gcode dispatcher to run a script");
            return;
        };
        let Ok(handle) = tokio::runtime::Handle::try_current() else {
            tracing::warn!("menu: running a menu script needs an async runtime");
            return;
        };
        handle.spawn(async move {
            for script in scripts {
                if let Err(error) = gcode.run_script(&script).await {
                    tracing::error!("menu: script error: {error}");
                }
            }
        });
    }
}

impl PrinterObject for MenuManager {
    /// `MenuManager.get_status` (`menu.py:754-760`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "timeout": self.timeout,
            "running": self.is_running(),
            "rows": self.rows,
            "cols": self.cols,
        })
    }
}

impl std::fmt::Debug for MenuManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MenuManager")
            .field("root", &self.root_ns)
            .field("items", &self.item_count())
            .field("running", &self.is_running())
            .field("stack", &self.stack_size())
            .finish_non_exhaustive()
    }
}

// ===========================================================================
// The shipped layout
// ===========================================================================

/// Parse the shipped layout (`MenuManager.load_config`, `menu.py:1030-1034`).
fn published_config() -> Result<Config, ConfigError> {
    Config::from_text(MENU_CFG)
        .map(|(config, _)| config)
        .map_err(|error| ConfigError::new(format!("Cannot load config '{MENU_CFG_NAME}': {error}")))
}

// ===========================================================================
// Option readers
// ===========================================================================

fn load_enable(config: &ConfigWrapper) -> Result<Enable, ConfigError> {
    let Some(text) = config.get_str("enable") else {
        return Ok(Enable::Bool(true));
    };
    match text.trim().to_ascii_lowercase().as_str() {
        "1" | "yes" | "true" | "on" => Ok(Enable::Bool(true)),
        "0" | "no" | "false" | "off" => Ok(Enable::Bool(false)),
        _ => Template::parse(&format!("{}:enable", config.identifier()), &text)
            .map(|template| Enable::Template(Arc::new(template)))
            .map_err(|error| ConfigError::new(error.to_string())),
    }
}

fn load_name(config: &ConfigWrapper, gcode_macro: &PrinterGCodeMacro) -> Result<Name, ConfigError> {
    let template = gcode_macro.load_template(config, "name", None)?;
    Ok(Name::Template(Arc::new(template)))
}

fn load_input(
    config: &ConfigWrapper,
    gcode_macro: &PrinterGCodeMacro,
) -> Result<InputData, ConfigError> {
    let value_tpl = gcode_macro.load_template(config, "input", None)?;
    let min_tpl = gcode_macro.load_template(config, "input_min", Some("-999999.0"))?;
    let max_tpl = gcode_macro.load_template(config, "input_max", Some("999999.0"))?;
    let step = config.get_float_bounded("input_step", Some(1.0), None, None, Some(0.0), None)?;
    let realtime = config.get_bool("realtime", Some(false))?;
    Ok(InputData {
        value_tpl: Arc::new(value_tpl),
        min_tpl: Arc::new(min_tpl),
        max_tpl: Arc::new(max_tpl),
        step,
        realtime,
        min: -999999.0,
        max: 999999.0,
        value: None,
        dirty: false,
        last_change: None,
    })
}

// ===========================================================================
// Name and namespace helpers
// ===========================================================================

/// The namespace a name refers to, from the item's own namespace
/// (`MenuElement.get_ns`, `menu.py:167-176`).
fn resolve_ns(ns: &str, name: &str) -> String {
    let name = name.trim();
    if let Some(rest) = name.strip_prefix("..") {
        return join(parent_ns(ns), rest.trim());
    }
    if let Some(rest) = name.strip_prefix('.') {
        return join(ns.to_string(), rest.trim());
    }
    name.to_string()
}

/// Join two namespaces with a space, skipping an empty half.
fn join(left: String, right: &str) -> String {
    if left.is_empty() {
        return right.trim().to_string();
    }
    if right.is_empty() {
        return left;
    }
    format!("{} {}", left.trim(), right.trim())
}

/// The parent namespace of `ns` (`get_ns('..')`).
fn parent_ns(ns: &str) -> String {
    let mut parts: Vec<&str> = ns.split(' ').filter(|part| !part.is_empty()).collect();
    parts.pop();
    parts.join(" ")
}

/// `MenuManager.asflat` (`menu.py:1060-1061`): join the lines, drop the quotes.
fn asflat(text: &str) -> String {
    stripliterals(&text.lines().collect::<Vec<_>>().join(""))
}

/// `MenuManager.stripliterals` (`menu.py:1055-1059`).
fn stripliterals(text: &str) -> String {
    let bytes = text.as_bytes();
    if bytes.len() >= 2 {
        let first = bytes[0];
        let last = bytes[bytes.len() - 1];
        if (first == b'"' && last == b'"') || (first == b'\'' && last == b'\'') {
            return text[1..text.len() - 1].to_string();
        }
    }
    text.to_string()
}

/// Cut `text` to `width` characters, the way a panel stops at its edge.
fn truncate_to(text: &str, width: usize) -> String {
    text.chars().take(width).collect()
}

/// Pad `text` with spaces to `width` characters (Python's `str.ljust`).
fn pad_to(text: &str, width: usize) -> String {
    let len = text.chars().count();
    if len >= width {
        return text.to_string();
    }
    let mut padded = text.to_string();
    padded.push_str(&" ".repeat(width - len));
    padded
}

/// Cut a name at `index` for the scroller (`__slice_name`, `menu.py:139-147`).
///
/// A `~glyph~` placeholder is one chunk; every other character is a chunk of
/// its own, which is what `chunks += text` in the Python does.
fn slice_name(name: &str, index: usize) -> String {
    let mut chunks: Vec<String> = Vec::new();
    let mut rest = name;
    while let Some(start) = rest.find('~') {
        let (plain, tail) = rest.split_at(start);
        for character in plain.chars() {
            chunks.push(character.to_string());
        }
        match tail[1..].find('~') {
            Some(end) => {
                chunks.push(tail[..end + 2].to_string());
                rest = &tail[end + 2..];
            }
            None => {
                for character in tail.chars() {
                    chunks.push(character.to_string());
                }
                rest = "";
            }
        }
    }
    for character in rest.chars() {
        chunks.push(character.to_string());
    }
    chunks.into_iter().skip(index).collect()
}

/// The truthiness of a rendered Python literal
/// (`bool(ast.literal_eval(text))`).
fn literal_truth(text: &str) -> Result<bool, MenuError> {
    let value = py_literal::parse(text.trim()).map_err(|error| error.to_string())?;
    Ok(match value {
        Value::Null => false,
        Value::Bool(value) => value,
        Value::Number(number) => number.as_f64().map(|value| value != 0.0).unwrap_or(true),
        Value::String(text) => !text.is_empty(),
        Value::Array(items) => !items.is_empty(),
        Value::Object(map) => !map.is_empty(),
    })
}

/// A rendered Python literal as a float (`float(ast.literal_eval(text))`).
fn literal_number(text: &str) -> Result<f64, MenuError> {
    let value = py_literal::parse(text.trim()).map_err(|error| error.to_string())?;
    match value {
        Value::Number(number) => number
            .as_f64()
            .ok_or_else(|| format!("not a number: {text}")),
        Value::Bool(value) => Ok(if value { 1.0 } else { 0.0 }),
        other => Err(format!("not a number: {other}")),
    }
}

/// Evaluate a number template without `menu.input` — what `MenuInput._eval_min`,
/// `_eval_max` and `_eval_value` do on `MenuElement.get_context`'s context.
fn eval_template_number(
    manager: &MenuManager,
    ns: &str,
    template: &Template,
) -> Result<f64, MenuError> {
    let mut context = manager.base_context();
    manager.bind_menu(&mut context, ns, None, None);
    let rendered = template
        .render(&mut context)
        .map_err(|error| error.to_string())?;
    literal_number(&rendered)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::gcode::{sync, CommandError, CommandHandler};
    use crate::core::klippy::reactor::ManualReactor;
    use crate::core::klippy::test_util::wait_until;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    /// The corpus's `lcd_type: st7920` shape.
    const ST7920_SECTION: &str = "lcd_type: st7920\ncs_pin: PA3\nsclk_pin: PA1\nsid_pin: PC1\n";

    fn machine() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// A machine with one display and the menu sections `sections`, whose
    /// `[display]` also carries `options`.
    fn load(options: &str, sections: &str) -> Arc<Printer> {
        let text =
            format!("[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}{options}\n{sections}");
        let config = Config::from_text(&text).expect("the config parses").0;
        let printer = machine();
        printer
            .load_config(&config)
            .unwrap_or_else(|error| panic!("the config loads: {error}"));
        printer
    }

    fn menu_of(printer: &Arc<Printer>) -> Arc<MenuManager> {
        printer
            .lookup_object_as::<MenuManager>(MENU_OBJECT)
            .expect("the menu object is registered")
    }

    /// The name of the row the cursor is on.
    fn selected_name(menu: &MenuManager) -> String {
        menu.stack_peek(0)
            .expect("a container is open")
            .selected_item()
            .expect("a row is selected")
            .name_text()
    }

    /// A temporary directory that removes itself on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-menu-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&path).expect("cannot create the test directory");
            Self(path)
        }

        fn path(&self) -> &std::path::Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    const TEST_TREE: &str = "[menu __test]\n\
                            type: list\n\
                            name: Test\n\
                            [menu __test __a]\n\
                            type: command\n\
                            name: A\n\
                            gcode: M117 a\n\
                            [menu __test __sub]\n\
                            type: list\n\
                            name: Sub\n\
                            [menu __test __sub __b]\n\
                            type: command\n\
                            name: B\n\
                            gcode: M117 b\n";

    // -- the tree -----------------------------------------------------------

    /// The shipped `menu.cfg` is a copy; this is the guard that says so.
    #[test]
    fn test_the_vendored_menu_has_not_drifted() {
        let upstream = klipperx_test_support::klipper_dir().join("klippy/extras/display/menu.cfg");
        let Ok(text) = std::fs::read_to_string(&upstream) else {
            return;
        };
        assert_eq!(
            MENU_CFG,
            text,
            "{} is out of date with the shipped menu.cfg",
            upstream.display()
        );
    }

    /// A main config's `[menu ...]` sections join the shipped tree, and a
    /// section the shipped file also defines replaces it in place.
    #[test]
    fn test_the_shipped_layout_and_the_main_config_build_one_tree() {
        let published = published_config()
            .expect("the shipped layout parses")
            .get_sections_by_id("menu")
            .len();
        assert_eq!(published, 82, "the shipped menu.cfg holds 82 items");

        let printer = load(
            "menu_root: __test\n",
            "[menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __cmd]\n\
             type: command\n\
             name: Cmd\n\
             gcode: M117 cmd\n\
             [menu __main __tune]\n\
             type: command\n\
             name: Overridden\n\
             gcode: M117 over\n",
        );
        let menu = menu_of(&printer);

        let root = menu.lookup_menuitem("__test").expect("the root item");
        assert!(root.is_container());
        assert_eq!(menu.lookup_children("__test"), vec!["__test __cmd"]);

        // The shipped tree is there, and the config's section replaced the
        // shipped `__main __tune` instead of adding a second one.
        assert!(menu.lookup_menuitem("__main __setup __dump").is_some());
        let tune = menu
            .lookup_menuitem("__main __tune")
            .expect("the tuned item");
        assert_eq!(tune.name_text(), "Overridden");
        let children = menu.lookup_children("__main");
        assert_eq!(
            children
                .iter()
                .filter(|name| *name == "__main __tune")
                .count(),
            1
        );
        // 82 shipped items, plus `__test` and `__test __cmd`; the override adds
        // no item of its own.
        assert_eq!(menu.item_count(), published + 2);
    }

    /// An unknown `menu_root` is refused, and so is an unknown item type.
    #[test]
    fn test_a_bad_root_and_a_bad_type_are_reported() {
        let text = format!("[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}menu_root: __nope\n");
        let config = Config::from_text(&text).expect("the config parses").0;
        let err = machine().load_config(&config).unwrap_err();
        assert_eq!(err.to_string(), "Unknown menuitem '__nope'");

        let text = format!(
            "[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}\n[menu __x]\ntype: nope\nname: X\n"
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let err = machine().load_config(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice 'nope' for option 'type' in section 'menu __x' is not a valid choice"
        );
    }

    // -- navigation ---------------------------------------------------------

    /// A click opens the menu; the arrows move the selection; a click on a
    /// container opens it, on a command runs it, on `..` steps back out.
    #[test]
    fn test_the_keys_open_move_and_press() {
        let printer = load("menu_root: __test\n", TEST_TREE);
        let menu = menu_of(&printer);
        assert!(!menu.is_running());

        // Not running: the first click opens the menu at the root.
        menu.key_event("click", 0.0);
        assert!(menu.is_running());
        assert_eq!(menu.stack_namespace(), vec!["__test"]);
        // A list's first row is its `..` item (`MenuList._populate`).
        assert_eq!(selected_name(&menu), "..");

        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "A");
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "Sub");
        menu.key_event("up", 0.0);
        assert_eq!(selected_name(&menu), "A");

        // Enter the container.
        menu.key_event("down", 0.0);
        menu.key_event("click", 0.0);
        assert_eq!(menu.stack_namespace(), vec!["__test", "__test __sub"]);
        assert_eq!(selected_name(&menu), "..");

        // Press the command: its script is queued, not run in place.
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "B");
        menu.key_event("click", 0.0);
        assert_eq!(menu.queued_gcode(), vec!["M117 b"]);

        // Back out: the parent reselects the container we were in.
        menu.key_event("back", 0.0);
        assert_eq!(menu.stack_namespace(), vec!["__test"]);
        assert_eq!(selected_name(&menu), "Sub");

        // The `..` row steps out too.
        menu.key_event("click", 0.0);
        assert_eq!(menu.stack_size(), 2);
        assert_eq!(selected_name(&menu), "..");
        menu.key_event("click", 0.0);
        assert_eq!(menu.stack_namespace(), vec!["__test"]);

        // Back at the root, back closes the menu.
        menu.key_event("back", 0.0);
        assert!(!menu.is_running());
    }

    /// `menu_reverse_navigation` swaps the two arrows (`menu.py:848-873`).
    #[test]
    fn test_reverse_navigation_swaps_the_arrows() {
        let printer = load(
            "menu_root: __test\nmenu_reverse_navigation: True\n",
            TEST_TREE,
        );
        let menu = menu_of(&printer);
        menu.key_event("click", 0.0);
        assert_eq!(selected_name(&menu), "..");
        menu.key_event("up", 0.0);
        assert_eq!(selected_name(&menu), "A");
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "..");
    }

    /// A disabled item is skipped, and an `enable` template is evaluated with
    /// the printer view.
    #[test]
    fn test_a_disabled_item_is_not_listed() {
        let printer = load(
            "menu_root: __test\n",
            "[menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __off]\n\
             type: command\n\
             name: Off\n\
             enable: False\n\
             gcode: M117 off\n\
             [menu __test __on]\n\
             type: command\n\
             name: On\n\
             enable: {1 == 1}\n\
             gcode: M117 on\n",
        );
        let menu = menu_of(&printer);
        menu.key_event("click", 0.0);
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "On");
        assert_eq!(menu.lookup_children("__test").len(), 2);
    }

    // -- inputs -------------------------------------------------------------

    /// `%3d` over a float renders what Python renders: `menu.cfg`'s input
    /// names are `{'%3d' % (menu.input*100)}%`, and `menu.input` is a float.
    #[test]
    fn test_a_printf_integer_conversion_truncates_a_float() {
        fn rendered(source: &str) -> String {
            let mut context = Context::new();
            context.insert("menu", Rt::Json(json!({"input": 9.99})));
            Template::parse("x", source)
                .expect("the format parses")
                .render(&mut context)
                .unwrap_or_else(|error| panic!("{error}"))
        }
        assert_eq!(
            rendered("Speed: {'%3d' % (menu.input*100)}%"),
            "Speed: 999%"
        );
        // A negative float truncates toward zero, as Python's `%d` does.
        assert_eq!(rendered("{'%d' % (0 - 3.7)}"), "-3");
        // An integer argument is untouched.
        assert_eq!(rendered("{'%03d' % 7}"), "007");
    }
    /// An input evaluates its `input`/`input_min`/`input_max` templates and
    /// steps by `input_step`, ten times as far when `fast_rate` applies.
    #[test]
    fn test_an_input_evaluates_and_steps() {
        let printer = load(
            "menu_root: __test\n",
            "[menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __speed]\n\
             type: input\n\
             name: Speed: {'%3d' % (menu.input*100)}%\n\
             input: 9.99\n\
             input_min: 0\n\
             input_max: 10\n\
             input_step: 0.01\n\
             realtime: True\n\
             gcode: M220 S{'%d' % (menu.input*100)}\n\
             [menu __test __low]\n\
             type: input\n\
             name: Low: {menu.input}\n\
             input: 0.005\n\
             input_min: 0\n\
             input_max: 10\n\
             input_step: 1\n\
             gcode: M117 low\n",
        );
        let menu = menu_of(&printer);
        let speed = menu.lookup_menuitem("__test __speed").expect("the input");

        // Not editing: the name renders the `input` template's value.
        assert_eq!(speed.name_text(), "Speed: 999%");
        assert_eq!(speed.input_value(), None);

        speed.start_editing();
        assert_eq!(speed.input_value(), Some(9.99));

        // The last step clamps at `input_max`.
        speed.inc_value(false);
        assert_eq!(speed.input_value(), Some(10.0));

        // `(max - min) / step > 100`, so a fast rate steps ten times as far.
        speed.dec_value(true);
        let value = speed.input_value().expect("still editing");
        assert!((value - 9.9).abs() < 1e-9, "{value}");
        speed.dec_value(false);
        let value = speed.input_value().expect("still editing");
        assert!((value - 9.89).abs() < 1e-9, "{value}");

        // Stop editing: the name goes back to the option's own value.
        speed.stop_editing();
        assert_eq!(speed.input_value(), None);
        assert_eq!(speed.name_text(), "Speed: 999%");

        // A step below the minimum clamps at it.
        let low = menu.lookup_menuitem("__test __low").expect("the input");
        low.start_editing();
        assert_eq!(low.input_value(), Some(0.005));
        low.dec_value(false);
        assert_eq!(low.input_value(), Some(0.0));
    }

    /// While an input is being edited the arrows change its value instead of
    /// moving the selection, and the `..` row is not reached.
    #[test]
    fn test_the_arrows_edit_an_input_in_place() {
        let printer = load(
            "menu_root: __test\n",
            "[menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __speed]\n\
             type: input\n\
             name: Speed: {menu.input}\n\
             input: 1\n\
             input_min: 0\n\
             input_max: 10\n\
             input_step: 1\n\
             gcode: M117 speed\n",
        );
        let menu = menu_of(&printer);
        menu.key_event("click", 0.0);
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "Speed: 1.0");

        // A click starts editing; the arrows then change the value.
        menu.key_event("click", 0.0);
        let speed = menu.lookup_menuitem("__test __speed").expect("the input");
        assert_eq!(speed.input_value(), Some(1.0));
        menu.key_event("down", 0.0);
        assert_eq!(speed.input_value(), Some(2.0));
        menu.key_event("up", 0.0);
        assert_eq!(speed.input_value(), Some(1.0));
        // Still the same row.
        assert_eq!(menu.selected_namespace().as_deref(), Some("__test __speed"));

        // A second click stops editing; the only other row is reached with the
        // up arrow (the list has just the `..` row and this input).
        menu.key_event("click", 0.0);
        assert_eq!(speed.input_value(), None);
        menu.key_event("up", 0.0);
        assert_eq!(selected_name(&menu), "..");
    }

    // -- the SD card --------------------------------------------------------

    /// A `vsdlist` lists the SD card's files as `M23 /<name>` commands
    /// (`MenuVSDList._populate`).
    #[test]
    fn test_a_vsdlist_lists_the_sd_cards_files() {
        let dir = TempDir::new("vsd");
        for name in ["a.gcode", "B.gcode", "notes.txt"] {
            std::fs::write(dir.path().join(name), "G28\n").expect("the file is written");
        }
        let sections = format!(
            "[virtual_sdcard]\npath: {}\n\
             [menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __files]\n\
             type: vsdlist\n\
             name: Files\n",
            dir.path().display()
        );
        let printer = load("menu_root: __test\n", &sections);
        let menu = menu_of(&printer);

        menu.key_event("click", 0.0);
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "Files");
        menu.key_event("click", 0.0);

        // Three files -- `a.gcode`, `B.gcode`, `notes.txt`, sorted
        // case-insensitively -- and the `..` row the list inserts first.
        let files = menu.stack_peek(0).expect("the list is open");
        assert_eq!(files.len(), 4);
        assert_eq!(selected_name(&menu), "..");
        menu.key_event("down", 0.0);
        assert_eq!(selected_name(&menu), "a.gcode");
        menu.key_event("click", 0.0);
        assert_eq!(menu.queued_gcode(), vec!["M23 /a.gcode"]);
    }

    // -- the script queue ---------------------------------------------------

    /// A pressed item's script is queued and then run as a g-code command with
    /// its parameters (`queue_gcode` / `dispatch_gcode`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_pressed_script_runs_as_a_gcode_command() {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let text = format!(
            "[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}menu_root: __test\n\
             [menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __cmd]\n\
             type: command\n\
             name: Cmd\n\
             gcode: MENU_TEST VALUE=7\n"
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        printer.load_config(&config).expect("the config loads");
        let menu = menu_of(&printer);

        let seen = Arc::new(Mutex::new(Vec::<(String, String)>::new()));
        let handler: CommandHandler = {
            let seen = Arc::clone(&seen);
            sync(move |gcmd: &crate::core::klippy::gcode::GcodeCommand| {
                let value = gcmd
                    .get_str("VALUE")
                    .map_err(|error| CommandError::new(error.to_string()))?;
                seen.lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push((gcmd.command().to_string(), value));
                Ok(())
            })
        };
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        gcode
            .register_command("MENU_TEST", handler, None, true)
            .expect("the command registers");

        menu.key_event("click", 0.0);
        menu.key_event("down", 0.0);
        menu.key_event("click", 0.0);
        assert_eq!(menu.queued_gcode(), vec!["MENU_TEST VALUE=7"]);

        // The queue's callback runs on the reactor; the script it dispatches
        // runs detached, so the test waits for the handler.
        reactor.run_due();
        let seen_by_handler = Arc::clone(&seen);
        let outcome = tokio::time::timeout(Duration::from_secs(10), async move {
            wait_until(
                Duration::from_secs(10),
                Duration::from_millis(10),
                || {
                    !seen_by_handler
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner())
                        .is_empty()
                },
                "the menu script reaches the dispatcher",
            )
            .await;
            seen_by_handler
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
        })
        .await
        .expect("the script runs");

        assert_eq!(outcome, vec![("MENU_TEST".to_string(), "7".to_string())]);
        assert!(menu.queued_gcode().is_empty());
    }

    // -- the rows -----------------------------------------------------------

    /// A row carries the cursor, the name padded to the panel's width, and the
    /// folder marker; a name too long for the row rolls after a heartbeat.
    #[test]
    fn test_a_long_name_rolls_and_a_container_is_marked() {
        let printer = load(
            "menu_root: __test\n",
            "[menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __long]\n\
             type: command\n\
             name: ABCDEFGHIJKLMNOPQRSTUVWXYZ\n\
             gcode: M117 long\n\
             [menu __test __sub]\n\
             type: list\n\
             name: Sub\n",
        );
        let menu = menu_of(&printer);
        assert_eq!(menu.cols(), 16);
        assert_eq!(menu.rows(), 4);

        menu.begin(0.0);
        // The `..` row first, then the long command.
        menu.down(false);
        let rows = menu.screen_rows_at(1.0);
        assert_eq!(rows.len(), 4, "a 16x4 panel shows four rows");
        assert_eq!(rows[1].prefix, ">");
        assert_eq!(rows[1].text, "ABCDEFGHIJKLMNO");
        assert_eq!(rows[1].suffix, "");
        // The container row is marked, and not selected.
        assert_eq!(rows[2].prefix, " ");
        assert_eq!(rows[2].text, pad_to("Sub", 14));
        assert_eq!(rows[2].suffix, ">");

        // The scroller was armed by the row above; the row-wide heartbeats then
        // cut one more character off the front every step.
        let rows = menu.screen_rows_at(1.6);
        assert_eq!(rows[1].text, "ABCDEFGHIJKLMNO");
        let rows = menu.screen_rows_at(2.2);
        assert_eq!(rows[1].text, "BCDEFGHIJKLMNOP");
    }

    /// The manager reports itself the way upstream's `get_status` does.
    #[test]
    fn test_the_menu_object_reports_its_status() {
        let printer = load("menu_root: __test\nmenu_timeout: 40\n", TEST_TREE);
        let menu = menu_of(&printer);
        let status = menu.get_status(0.0);
        assert_eq!(status["timeout"], json!(40));
        assert_eq!(status["running"], json!(false));
        assert_eq!(status["rows"], json!(4));
        assert_eq!(status["cols"], json!(16));

        // The display hands the screen to the menu while it is open.
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        assert!(!menu.screen_update_event(0.0));
        menu.key_event("click", 0.0);
        assert!(menu.screen_update_event(0.0));
        assert_eq!(display.get_dimensions(), (16, 4));
    }
}
