//! `pulse_counter` — GPIO edge counters.
//!
//! Upstream's `klippy/extras/pulse_counter.py`: `MCU_counter` takes an oid on
//! the pin's MCU, asks the firmware to poll that pin, and hands each
//! `counter_state` report to a callback; `FrequencyCounter` turns those reports
//! into a frequency in Hertz. The one consumer is the fan tachometer
//! ([`fan`](crate::core::klippy::extras::fan)'s `tachometer_pin`), which scales
//! the frequency into RPM.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_counter oid=%c pin=%u pull_up=%c` |
//! | host → MCU | `query_counter oid=%c clock=%u poll_ticks=%u sample_ticks=%u` |
//! | MCU → host | `counter_state oid=%c next_clock=%u count=%u count_clock=%u` |
//!
//! `config_counter` describes the pin and belongs to the **config** list.
//! `query_counter` arms the poll timer: upstream adds it to the *init* list
//! from `MCU_counter.build_config`, but it carries an absolute clock, and this
//! host's init list is re-sent after a firmware reset whose clock has since
//! restarted — so it is armed from the **post-init** callback instead, with a
//! fresh query slot, the way [`ds18b20`](crate::core::klippy::extras::ds18b20)
//! arms its query. The `counter_state` report is bound at the same point: one
//! callback per message name routes every counter on the MCU by oid, which is
//! what upstream gets from `register_serial_response(..., oid=…)`.
//!
//! The count on the wire is 32 bits and wraps; the host carries it into 64 bits
//! (`MCU_counter._handle_counter_state`), and [`FrequencyCounter`] differences
//! consecutive samples (`FrequencyCounter._counter_callback`).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, Weak};

use tracing::warn;

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::mcu::{pin_number, query_slot, Mcu, McuChip, McuError};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::pins::PrinterPins;

/// `config_counter oid=%c pin=%u pull_up=%c` (`src/pulse_counter.c:61`): the
/// firmware allocates the counter and sets up the GPIO input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ConfigCounter {
    /// The oid this counter was given at build time.
    oid: u8,
    /// The pin as the firmware's `pin` enumeration numbers it.
    pin: u32,
    /// `^` → 1, `~` → -1, bare → 0 (`pins.parse_pin`).
    pull_up: i8,
}

impl McuCommand for ConfigCounter {
    const NAME: &'static str = "config_counter";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.pin),
            ArgValue::UInt8(self.pull_up as u8),
        ]
    }
}

/// `query_counter oid=%c clock=%u poll_ticks=%u sample_ticks=%u`
/// (`src/pulse_counter.c:74`): arm the poll timer and the report period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct QueryCounter {
    /// The counter's oid.
    oid: u8,
    /// Absolute firmware clock of the first poll.
    clock: u32,
    /// Ticks between polls of the pin.
    poll_ticks: u32,
    /// Ticks between two `counter_state` reports.
    sample_ticks: u32,
}

impl McuCommand for QueryCounter {
    const NAME: &'static str = "query_counter";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.poll_ticks),
            ArgValue::UInt32(self.sample_ticks),
        ]
    }
}

/// `counter_state oid=%c next_clock=%u count=%u count_clock=%u`
/// (`src/pulse_counter.c:128`) — one periodic sample of the edge count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterState {
    /// The counter this report belongs to.
    pub oid: u8,
    /// The clock of the *next* poll; the sample is dated one poll earlier.
    pub next_clock: u32,
    /// The 32-bit edge count as the firmware last read it.
    pub count: u32,
    /// The clock at which `count` was read.
    pub count_clock: u32,
}

impl McuResponse for CounterState {
    const NAME: &'static str = "counter_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            next_clock: params.get_u32("next_clock")?,
            count: params.get_u32("count")?,
            count_clock: params.get_u32("count_clock")?,
        })
    }
}

/// What the counter feeds: upstream's `(time, count, count_time)` triple.
type CounterCallback = Box<dyn Fn(f64, u64, f64) + Send + Sync>;

/// One counter's shared state: which MCU it lives on, its oid, and the report
/// callback.
///
/// Held behind an [`Arc`] because the [`CounterRegistry`] routes reports to it
/// weakly — the counter's owner holds the strong reference.
struct CounterCore {
    /// The MCU the pin belongs to: it owns the oid and maps the report clocks.
    chip: McuChip,
    /// The oid the firmware assigned.
    oid: u8,
    /// Ticks between the firmware's polls, fixed when the config was built
    /// (`MCU_counter._poll_ticks`).
    poll_ticks: Mutex<u32>,
    /// The edge count carried across the firmware's 32-bit wrap
    /// (`MCU_counter._last_count`).
    count: Mutex<u64>,
    /// Upstream's `_callback`, installed through [`McuCounter::setup_callback`].
    callback: Mutex<Option<CounterCallback>>,
}

impl CounterCore {
    /// One `counter_state` report: map the two firmware clocks back to print
    /// time, then carry the count across the wrap and hand it on
    /// (`MCU_counter._handle_counter_state`).
    fn handle(&self, values: &[ArgValue]) {
        let (next_clock, count, count_clock) = match (values.get(1), values.get(2), values.get(3)) {
            (
                Some(ArgValue::UInt32(next_clock)),
                Some(ArgValue::UInt32(count)),
                Some(ArgValue::UInt32(count_clock)),
            ) => (*next_clock, *count, *count_clock),
            _ => return,
        };
        // The sample is dated at the next poll moved back one poll period, as
        // upstream does (`time = clock_to_print_time(next_clock - poll_ticks)`).
        let Some(next) = self.chip.clock32_to_clock64(next_clock) else {
            return;
        };
        let poll_ticks = i64::from(lock(&self.poll_ticks));
        let Some(time) = self.chip.clock_to_print_time(next - poll_ticks) else {
            return;
        };
        let Some(count_clock) = self.chip.clock32_to_clock64(count_clock) else {
            return;
        };
        let Some(count_time) = self.chip.clock_to_print_time(count_clock) else {
            return;
        };
        self.dispatch(time, count, count_time);
    }

    /// Carry `count` across the 32-bit wrap and pass it to the callback
    /// (`MCU_counter._handle_counter_state`, second half).
    fn dispatch(&self, time: f64, count: u32, count_time: f64) {
        let count = {
            let mut last = lock(&self.count);
            // `(count - last) & 0xffffffff`, added back to `last`: the
            // difference is taken mod 2^32, so a wrapped count lands *past*
            // `last` instead of below it, and the sum keeps growing.
            *last += u64::from(count.wrapping_sub(*last as u32));
            *last
        };
        // The count lock is released first: a callback may read the frequency,
        // which takes its own lock.
        let callback = lock(&self.callback);
        if let Some(callback) = callback.as_ref() {
            callback(time, count, count_time);
        }
    }
}

/// `MCU_counter`: one edge counter on one pin of one MCU.
///
/// The construction half of upstream's class: it looks the pin up (with pull-up
/// support, without inversion — `lookup_pin(pin, can_pullup=True)`), takes an
/// oid from that MCU, and registers the two callbacks that speak to the
/// firmware. [`FrequencyCounter`] is what a consumer normally builds.
pub struct McuCounter {
    /// The shared state the report registry routes to; kept so the counter (and
    /// its registration) outlives its reports.
    core: Arc<CounterCore>,
}

impl McuCounter {
    /// Build the counter for `pin_desc` and register its callbacks
    /// (`MCU_counter.__init__` + `build_config`).
    ///
    /// `sample_time` is the report period and `poll_time` the period the
    /// firmware samples the pin at — both in seconds.
    ///
    /// # Errors
    /// The pin cannot be looked up, names a chip that is not an MCU, or this
    /// MCU cannot hand out an oid or a callback slot.
    pub fn new(
        pins: &PrinterPins,
        pin_desc: &str,
        sample_time: f64,
        poll_time: f64,
    ) -> Result<Self, ConfigError> {
        let params = pins
            .lookup_pin(pin_desc, false, true, None)
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let chip = pins.chip_as::<McuChip>(&params.chip_name).ok_or_else(|| {
            ConfigError::new(format!(
                "Pin chip '{}' is not an MCU (tachometer_pin)",
                params.chip_name
            ))
        })?;
        let oid = chip
            .config()
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let core = Arc::new(CounterCore {
            chip: chip.clone(),
            oid,
            poll_ticks: Mutex::new(0),
            count: Mutex::new(0),
            callback: Mutex::new(None),
        });

        // Build time: turn the pin description into a number, announce the
        // counter, and remember the poll period the report dates itself with.
        let build_core = Arc::clone(&core);
        let build_chip = chip.clone();
        let build_params = params.clone();
        chip.config()
            .register_config_callback(Box::new(move |builder, mcu| {
                let pin = pin_number(
                    mcu,
                    &build_chip.resolve_pin(&build_params.pin)?,
                    build_chip.name(),
                )?;
                builder.add_config_cmd(&ConfigCounter {
                    oid,
                    pin,
                    pull_up: build_params.pullup,
                })?;
                *lock(&build_core.poll_ticks) = mcu.seconds_to_clock(poll_time)? as u32;
                Ok(())
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // Post-init: the firmware has the oid now, so arm the poll timer with a
        // fresh clock and bind the reports (see the module docs).
        let arm_core = Arc::clone(&core);
        chip.config()
            .register_post_init_callback(Box::new(move |mcu| {
                match arm_query(&arm_core, mcu, sample_time) {
                    Ok(query) => {
                        if let Err(err) = mcu.send_msg(&query) {
                            warn!(
                                "MCU '{}': could not arm the pulse counter: {err}",
                                mcu.name()
                            );
                        }
                    }
                    Err(err) => warn!(
                        "MCU '{}': could not arm the pulse counter: {err}",
                        mcu.name()
                    ),
                }
                let registry = registry_for(mcu.name());
                if let Err(err) = registry.bind(mcu, arm_core.oid, Arc::downgrade(&arm_core)) {
                    warn!(
                        "MCU '{}': could not bind the pulse counter report: {err}",
                        mcu.name()
                    );
                }
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        Ok(Self { core })
    }

    /// Install the callback each report is handed to
    /// (`MCU_counter.setup_callback`).
    ///
    /// It runs once per `counter_state` report with the same `(time, count,
    /// count_time)` upstream passes — including the first one, before anything
    /// can be differenced against it.
    pub fn setup_callback(&self, callback: impl Fn(f64, u64, f64) + Send + Sync + 'static) {
        *lock(&self.core.callback) = Some(Box::new(callback));
    }

    /// The oid the firmware assigned to this counter.
    pub fn oid(&self) -> u8 {
        self.core.oid
    }
}

/// The `query_counter` this counter arms with (`MCU_counter.build_config`'s
/// init command), clocked from the estimate taken now.
fn arm_query(core: &CounterCore, mcu: &Mcu, sample_time: f64) -> Result<QueryCounter, McuError> {
    Ok(QueryCounter {
        oid: core.oid,
        clock: query_slot(mcu, core.oid)?,
        poll_ticks: lock(&core.poll_ticks),
        sample_ticks: mcu.seconds_to_clock(sample_time)? as u32,
    })
}

/// `FrequencyCounter`: a [`McuCounter`] reduced to Hertz.
///
/// Upstream's class of the same name: every report differences the count and
/// the time against the previous one, so the frequency is the edge rate over
/// the last sample period. The first report only anchors the pair — there is
/// nothing to difference against yet — and a report that carries no later time
/// reads as zero rather than as an infinite rate.
pub struct FrequencyCounter {
    /// Kept alive so its registry entry and callbacks outlive the readings.
    counter: McuCounter,
    /// The running `(time, count, frequency)` the callback maintains.
    state: Arc<FrequencyState>,
}

impl FrequencyCounter {
    /// Build the counter for `pin_desc`, sampling every `sample_time` seconds
    /// and polling the pin every `poll_time` seconds
    /// (`FrequencyCounter.__init__`).
    ///
    /// # Errors
    /// As [`McuCounter::new`].
    pub fn new(
        pins: &PrinterPins,
        pin_desc: &str,
        sample_time: f64,
        poll_time: f64,
    ) -> Result<Self, ConfigError> {
        let counter = McuCounter::new(pins, pin_desc, sample_time, poll_time)?;
        let state = Arc::new(FrequencyState::default());
        let callback_state = Arc::clone(&state);
        counter.setup_callback(move |time, count, count_time| {
            callback_state.sample(time, count, count_time);
        });
        Ok(Self { counter, state })
    }

    /// The last computed frequency in Hertz (upstream's `get_frequency`).
    pub fn get_frequency(&self) -> f64 {
        lock(&self.state.numbers).freq
    }
}

/// The numbers behind [`FrequencyCounter`]: upstream's `_last_time`,
/// `_last_count` and `_freq`.
#[derive(Default)]
struct FrequencyNumbers {
    /// The time the previous sample was dated at (`None` until the first one).
    last_time: Option<f64>,
    /// The count the previous sample carried.
    last_count: u64,
    /// The most recent frequency in Hertz.
    freq: f64,
}

/// [`FrequencyNumbers`], behind the lock [`FrequencyCounter`] reads through.
#[derive(Default)]
struct FrequencyState {
    numbers: Mutex<FrequencyNumbers>,
}

impl FrequencyState {
    /// Fold one report into the frequency
    /// (`FrequencyCounter._counter_callback`).
    fn sample(&self, time: f64, count: u64, count_time: f64) {
        let mut numbers = lock(&self.numbers);
        let Some(last_time) = numbers.last_time else {
            // First sample: anchor the pair, report nothing.
            numbers.last_time = Some(time);
            numbers.last_count = count;
            return;
        };
        let delta_time = count_time - last_time;
        if delta_time > 0. {
            numbers.last_time = Some(count_time);
            // The count only grows: the wrap was carried before this point.
            numbers.freq = (count - numbers.last_count) as f64 / delta_time;
        } else {
            // No time since the last sample: there is no rate to read.
            numbers.last_time = Some(time);
            numbers.freq = 0.;
        }
        numbers.last_count = count;
    }
}

/// Per-oid routing for `counter_state`, one registry per MCU.
///
/// One callback can be bound per message name, so the counters on an MCU share
/// a registry and one bound closure routes by oid — the same thing upstream's
/// `register_serial_response(..., oid=…)` does. Registries are keyed by MCU
/// name because a counter is built before its MCU is connected.
#[derive(Default)]
struct CounterRegistry {
    counters: Mutex<HashMap<u8, Weak<CounterCore>>>,
}

impl CounterRegistry {
    /// Add `core` and route this MCU's `counter_state` reports to it.
    ///
    /// A later bind replaces the closure, which is harmless: every counter on
    /// the MCU is in the same table by then.
    fn bind(self: &Arc<Self>, mcu: &Mcu, oid: u8, core: Weak<CounterCore>) -> Result<(), McuError> {
        lock(&self.counters).insert(oid, core);
        let registry = Arc::clone(self);
        mcu.bind_callback(CounterState::NAME, move |values| registry.route(values))
    }

    /// Hand one report to the counter its `oid` names (the closure
    /// [`CounterRegistry::bind`] installs).
    fn route(&self, values: &[ArgValue]) {
        // Every report starts with `oid=%c`; anything else is not ours.
        let oid = match values.first() {
            Some(ArgValue::UInt8(oid)) => *oid,
            _ => return,
        };
        let core = lock(&self.counters).get(&oid).and_then(Weak::upgrade);
        if let Some(core) = core {
            core.handle(values);
        }
    }
}

/// The registry for one MCU name, created on first use.
fn registry_for(name: &str) -> Arc<CounterRegistry> {
    static REGISTRIES: OnceLock<Mutex<HashMap<String, Arc<CounterRegistry>>>> = OnceLock::new();
    let registries = REGISTRIES.get_or_init(|| Mutex::new(HashMap::new()));
    let mut registries = lock(registries);
    Arc::clone(
        registries
            .entry(name.to_string())
            .or_insert_with(|| Arc::new(CounterRegistry::default())),
    )
}

/// One lock acquisition, poisoned or not: these guards only ever cover single
/// assignments and reads, so a poison bit says nothing this code can act on.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poison| poison.into_inner())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::clock::McuClock;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// The messages as the built dictionaries declare them
    /// (`test/configs` → `stm32f407.dict`), with the pin enumeration that
    /// resolves the tachometer pins.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 8,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_counter oid=%c pin=%u pull_up=%c": 64,
                "query_counter oid=%c clock=%u poll_ticks=%u sample_ticks=%u": 63
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": -1,
                "counter_state oid=%c next_clock=%u count=%u count_clock=%u": 143
            },
            "enumerations": {"pin": {"PC0": 32, "PE10": 74}},
            "config": {"CLOCK_FREQ": 168000000}
        }))
        .expect("the counter dictionary parses")
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).expect("it installs");
        parser
    }

    /// A chip registered as `<name>` over a test MCU whose clock is seeded.
    ///
    /// The name is unique per test: the report registry is a process-wide map
    /// keyed by MCU name, and two tests sharing one would route each other's
    /// reports.
    fn chip() -> (McuChip, Arc<PrinterPins>, Arc<Mcu>, String) {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let name = format!("counter_mcu{}", NEXT.fetch_add(1, Ordering::Relaxed));
        let mcu = Arc::new(Mcu::for_test(
            name.clone(),
            Interface::new(FrameMock::new(Vec::new())),
        ));
        mcu.install_dictionary(dictionary()).expect("it installs");
        // The query slot reads the clock estimate taken at connect.
        mcu.set_clock_base(1_000_000);
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            name.clone(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip(&name, Arc::new(chip.clone()))
            .expect("the name is free");
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(Arc::clone(&mcu));
        (chip, pins, mcu, name)
    }

    /// The decoded `(name, args)` of one encoded list.
    fn decode_list(
        payloads: &[crate::core::klippy::msg::proto::Payload],
    ) -> Vec<(String, Vec<ArgValue>)> {
        let mut parser = parser();
        payloads
            .iter()
            .map(|payload| {
                let frame = Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).expect("it decodes");
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    /// A report carrying `count` at `count_clock`, next poll at `next_clock`.
    fn report(oid: u8, count: u32) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(oid),
            ArgValue::UInt32(168_000_000),
            ArgValue::UInt32(count),
            ArgValue::UInt32(167_000_000),
        ]
    }

    /// `1.0 s` of the seeded clock minus the 1.5 ms poll period.
    fn expected_time() -> f64 {
        (168_000_000 - 252_000) as f64 / 168_000_000.
    }

    /// `0.994 s` of the seeded clock.
    fn expected_count_time() -> f64 {
        167_000_000 as f64 / 168_000_000.
    }

    #[test]
    fn test_the_commands_round_trip_through_the_firmware_format() {
        let parser = parser();

        let config = ConfigCounter {
            oid: 3,
            pin: 74,
            pull_up: -1,
        };
        let encoded = parser
            .encode(ConfigCounter::NAME, &config.args())
            .expect("the dictionary declares it");
        assert_eq!(
            parser.decode(encoded).expect("it decodes")[0].1,
            config.args()
        );

        let query = QueryCounter {
            oid: 3,
            clock: 253_000_000,
            poll_ticks: 252_000,
            sample_ticks: 168_000_000,
        };
        let encoded = parser
            .encode(QueryCounter::NAME, &query.args())
            .expect("the dictionary declares it");
        assert_eq!(
            parser.decode(encoded).expect("it decodes")[0].1,
            query.args()
        );
    }

    #[test]
    fn test_the_config_describes_the_pin_and_leaves_arming_to_post_init() {
        let (chip, pins, mcu, name) = chip();
        McuCounter::new(&pins, &format!("{name}:^PC0"), 1.0, 0.0015).expect("it builds");

        let built = chip.config().build(&mcu).expect("it builds");
        let config = decode_list(&built.config);
        let counter = config
            .iter()
            .find(|(name, _)| name == ConfigCounter::NAME)
            .expect("config_counter is queued");
        // `^PC0` → pin 32 with the pull-up set.
        assert_eq!(
            counter.1,
            vec![ArgValue::UInt8(0), ArgValue::UInt32(32), ArgValue::UInt8(1)]
        );
        // The query carries an absolute clock, so it is armed from post-init
        // rather than baked into `init` (a waketime from before a firmware
        // reset would be off the new clock).
        assert!(built.init.is_empty(), "{:?}", built.init);
    }

    #[test]
    fn test_the_query_slot_carries_both_tick_periods() {
        let (chip, pins, mcu, name) = chip();
        let counter =
            McuCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");
        // The build runs the config callback, which fixes the poll period.
        chip.config().build(&mcu).expect("it builds");
        let query = arm_query(&counter.core, &mcu, 1.0).expect("it arms");

        assert_eq!(query.oid, counter.oid());
        assert_eq!(query.poll_ticks, 252_000, "poll every 1.5 ms at 168 MHz");
        assert_eq!(query.sample_ticks, 168_000_000, "report every second");
        // The slot is 1.5 s past the seeded estimate, plus `oid * 10 ms`.
        assert!(
            (253_000_000..263_000_000).contains(&u64::from(query.clock)),
            "query slot {} is ~1.5 s past the seeded estimate",
            query.clock
        );
    }

    #[test]
    fn test_reports_route_to_the_counter_their_oid_names() {
        let (chip, pins, mcu, name) = chip();
        let first = McuCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");
        let second =
            McuCounter::new(&pins, &format!("{name}:PE10"), 1.0, 0.0015).expect("it builds");
        // The build runs both config callbacks, which fix the poll period the
        // report dates itself with.
        chip.config().build(&mcu).expect("it builds");

        let samples = Arc::new(Mutex::new(Vec::new()));
        let first_seen = Arc::clone(&samples);
        first.setup_callback(move |time, count, count_time| {
            first_seen.lock().unwrap().push((time, count, count_time));
        });
        let second_count = Arc::new(Mutex::new(0u64));
        let seen_count = Arc::clone(&second_count);
        second.setup_callback(move |_time, count, _count_time| {
            *seen_count.lock().unwrap() = count;
        });

        let registry = registry_for(&name);
        registry
            .bind(&mcu, first.oid(), Arc::downgrade(&first.core))
            .expect("the dictionary declares counter_state");
        registry
            .bind(&mcu, second.oid(), Arc::downgrade(&second.core))
            .expect("a later bind replaces the closure");

        registry.route(&report(second.oid(), 9));
        assert_eq!(
            *second_count.lock().unwrap(),
            9,
            "the second counter got it"
        );
        assert!(samples.lock().unwrap().is_empty(), "the first did not");

        registry.route(&report(first.oid(), 5));
        let samples = samples.lock().unwrap();
        assert_eq!(samples.len(), 1, "the first counter got its report");
        let (time, count, count_time) = samples[0];
        assert_eq!(count, 5);
        assert!((time - expected_time()).abs() < 1e-9, "{time}");
        assert!(
            (count_time - expected_count_time()).abs() < 1e-9,
            "{count_time}"
        );
    }

    #[test]
    fn test_a_wrapped_count_carries_into_the_next_bit() {
        let (_chip, pins, _mcu, name) = chip();
        let counter =
            McuCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");

        counter.core.dispatch(0.0, 0xffff_fffe, 0.0);
        counter.core.dispatch(1.0, 3, 1.0);

        assert_eq!(
            *lock(&counter.core.count),
            0x1_0000_0003,
            "the wrapped count continued past 2^32 instead of resetting"
        );
    }

    #[test]
    fn test_every_report_reaches_the_callback_including_the_first() {
        let (_chip, pins, _mcu, name) = chip();
        let counter =
            McuCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");
        let seen = Arc::new(Mutex::new(Vec::new()));
        let callback_seen = Arc::clone(&seen);
        counter.setup_callback(move |time, count, count_time| {
            callback_seen
                .lock()
                .unwrap()
                .push((time, count, count_time));
        });

        counter.core.dispatch(0.0, 7, 0.0);
        counter.core.dispatch(1.0, 12, 1.0);

        let seen = seen.lock().unwrap();
        assert_eq!(seen.len(), 2, "the first report is not swallowed: {seen:?}");
        assert_eq!(seen[0].1, 7);
        assert_eq!(seen[1].1, 12);
    }

    #[test]
    fn test_the_first_sample_only_anchors_the_time() {
        let (_chip, pins, _mcu, name) = chip();
        let counter =
            FrequencyCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");
        assert_eq!(counter.get_frequency(), 0.0);

        counter.counter.core.dispatch(0.0, 10, 0.0);

        assert_eq!(counter.get_frequency(), 0.0, "nothing to difference yet");
    }

    #[test]
    fn test_the_frequency_is_the_count_delta_over_the_time_delta() {
        let (_chip, pins, _mcu, name) = chip();
        let counter =
            FrequencyCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");

        counter.counter.core.dispatch(0.0, 10, 0.0);
        counter.counter.core.dispatch(1.0, 30, 1.0);

        // 20 edges in the second that ended at `count_time = 1.0`.
        assert_eq!(counter.get_frequency(), 20.0);
    }

    #[test]
    fn test_a_sample_with_no_new_time_reads_zero() {
        let (_chip, pins, _mcu, name) = chip();
        let counter =
            FrequencyCounter::new(&pins, &format!("{name}:PC0"), 1.0, 0.0015).expect("it builds");

        counter.counter.core.dispatch(0.0, 10, 0.0);
        counter.counter.core.dispatch(1.0, 30, 1.0);
        // `count_time` did not advance past the previous sample's time, so
        // there is no window to divide by (`_freq = 0.`) — and the anchor moves
        // to this report's own time.
        counter.counter.core.dispatch(2.0, 31, 1.0);
        assert_eq!(counter.get_frequency(), 0.0);

        // The next sample measures against that anchor: 19 edges in 1 s.
        counter.counter.core.dispatch(3.0, 50, 3.0);
        assert_eq!(counter.get_frequency(), 19.0);
    }

    #[test]
    fn test_a_tachometer_pin_may_pull_up_but_may_not_invert() {
        let (_chip, pins, _mcu, name) = chip();

        FrequencyCounter::new(&pins, &format!("{name}:^PC0"), 1.0, 0.0015)
            .expect("a pull-up is allowed");

        let err = FrequencyCounter::new(&pins, &format!("{name}:!PC0"), 1.0, 0.0015)
            .expect_err("upstream's lookup_pin cannot invert");
        assert!(err.to_string().contains("Invalid pin"), "{err}");
    }
}
