//! `klipperx stress` — bench the MCU: ramp its step or link load until it fails,
//! or run one full host motion path as a smoke test.
//!
//! `--task step` and `--task comm` are the ramps. `--task step`'s workload is
//! upstream's step engine (`src/stepper.c`): the host configures a firmware
//! stepper for every stepper section the board owns (borrowing each section's
//! step/dir pins) and then queues `queue_step` moves at a rising step rate —
//! one ramp, driven onto all of that board's steppers at once. The MCU stops
//! being able to keep up with a firmware shutdown:
//!
//! * the next step's time has already passed — `Stepper too far in past`
//!   (`src/stepper.c:108`), the usual one;
//! * a new timer's time has already passed — `Timer too close` (`src/sched.c:94`);
//! * more moves were queued than the move queue holds — `Move queue overflow`
//!   (`src/basecmd.c:90`).
//!
//! So the ramp's job is to find the step rate just before that. Each stage drives
//! one rate for [`STAGE_SECONDS`] as a steady stream of `queue_step` moves, waits
//! for it to run out (and for the stepper to actually stop), and asks the firmware
//! whether it is still alive. The first rate that leaves it shut down is the
//! answer; the previous one is the last it survived. Because a shutdown can only
//! be cleared by a `reset`, the ramp only goes up — the result is a bracket, kept
//! tight by making each stage a small multiple of the one before.
//!
//! This is a **bench tool**, not part of the host: it takes the MCU over (the
//! config handshake resets a board carrying a different configuration) and leaves
//! it shut down when it finds the limit. A real `printer.cfg` is not required —
//! only the `[mcu <name>]` section (for the transport) and the stepper sections
//! whose `step_pin` is on that board (for step/dir pin pairs) are read.
//!
//! `--task motion` is the odd one out: a single, bounded move through the real
//! host stack ([`Trapq`] → [`Stepper`] → the full compressor) with the firmware's
//! `stepper_get_position` read back, so FW5f's compression can be checked on
//! hardware without a full `[printer]` config or three known axes.
//!
//! By default a run drives one board: the MCU name argument omitted or empty is
//! the bare `[mcu]`. The name can be repeated to drive several boards at once,
//! and `--all-mcus` takes every `[mcu]` section in the config (the bare `[mcu]`
//! counts as one). Each board gets its own connection and its own ramp, run
//! concurrently on the same runtime, and every report line carries its board's
//! `[<name>]` prefix so the interleaved output stays attributable. A board that
//! fails makes the command fail; a board that reaches its limit counts as a
//! success, exactly as for a single board. A board `--all-mcus` found that owns
//! no stepper section is skipped with a line of its own and leaves the exit
//! code alone (a board the command *named* is promised a stepper and fails
//! instead); a run in which no selected board could drive a stepper fails,
//! because nothing was tested.

use clap::Args;
use std::future::{poll_fn, Future};
use std::mem::take;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::task::Poll;
use std::time::{Duration, Instant};

use crate::core::klippy::cmd::config::{ConfigState, GetConfig, Reset};
use crate::core::klippy::cmd::stepper::{
    QueueStep, ResetStepClock, SetNextStepDir, StepperGetPosition, StepperPosition,
};
use crate::core::klippy::cmd::uptime::{GetUptime, Uptime};
use crate::core::klippy::cmd::{GetClock, McuCommand};
use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigWrapper};
use crate::core::klippy::extras::board_pins::BoardPins;
use crate::core::klippy::mathutil::Xyz;
use crate::core::klippy::mcu::{pin_number, ConfigBuilder, Mcu, McuChip, McuError};
use crate::core::klippy::motion::{Axis, StepCommand, Stepper, Trapq};
use crate::core::klippy::pins::{PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;
use crate::core::klippy::reactor::ManualReactor;

/// The first step rate to try, in steps per second.
///
/// Low enough that any Klipper MCU should survive it, so the ramp starts from a
/// known-good point.
const START_RATE: f64 = 10_000.0;

/// Each stage is this many times the previous one.
///
/// This is what sets the accuracy of the answer: the run brackets the limit
/// between the last rate that survived and the first that did not, so a smaller
/// step is a tighter bracket and a longer run.
const RATE_STEP: f64 = 1.25;

/// Stop here; a firmware that survives this is reported as "no failure found".
const MAX_RATE: f64 = 20_000_000.0;

/// How long each stage drives the stepper, in seconds.
///
/// Long enough that a rate just above the limit has time to fall behind far
/// enough to shut the firmware down, which is what makes a marginal rate show up
/// as a failure instead of a pass.
const STAGE_SECONDS: f64 = 0.5;

/// Each `queue_step` command covers about this much time.
///
/// Sizing the commands by time rather than by a fixed count keeps them uniform:
/// the firmware sees a steady stream of similar moves at every rate.
const SLICE_SECONDS: f64 = 0.01;

/// Move-queue entries reserved **per** stress stepper, and the most one stage
/// queues on it. The firmware's own queue is larger (the board this was tried on
/// reported 1024); this only has to hold one stage's worth.
const MOVE_SLOTS: u32 = 64;

/// How many `queue_step` commands to queue before letting the send task drain.
///
/// The outbound channel holds 512 items (`SEND_QUEUE_CAPACITY` in `mcu/mod.rs`;
/// the sync sender waits bounded when full), so a stage's commands go
/// out in batches that stay under it, with a flush between. The flush paces the
/// *sending*, not the stepping: the commands still chain, so the schedule the
/// firmware sees is the same.
const SEND_BATCH: u32 = 16;

/// The first request rate the comm task tries, in requests per second.
const COMM_START_RATE: f64 = 100.0;

/// The comm ramp stops here; a link that carries it is reported as such.
const COMM_MAX_RATE: f64 = 200_000.0;

/// Unanswered requests allowed before the link counts as losing them.
///
/// In flight there is the outbound channel (512 items) plus the wire, so this is
/// well above what a healthy link holds at any moment.
const COMM_BACKLOG_LIMIT: u64 = 128;

/// What the tool drives.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Task {
    /// Step generation: the highest step rate the MCU can hold (default).
    Step,
    /// Command round-trips: the highest request rate the host↔MCU link carries.
    Comm,
    /// Full-compressor smoke test: drive one stepper through `G1`'s host path
    /// (`Trapq` → `Stepper` → `stepcompress`) and read the position back.
    Motion,
}

/// A `get_config` / `get_clock` round-trip that takes longer than this is
/// treated as the MCU no longer answering.
const CALL_TIMEOUT: Duration = Duration::from_secs(2);

/// How long to wait before reopening a board that was told to `reset`, and how
/// many times to try. The same shape as `McuObject`'s reconnect path.
const RECONNECT_DELAY: Duration = Duration::from_millis(250);
const RECONNECT_ATTEMPTS: usize = 20;

/// How long to give the `reset` command's flush before reopening.
const RESET_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

/// How long to let the flushed `reset` reach the firmware before the old
/// session's port is closed — upstream's pause between sending `reset` and
/// `_disconnect()` (`klippy/mcu.py`). The flush only proves the bytes left the
/// host; closing immediately could cut a write the tty driver still holds, and
/// a firmware that never saw `reset` never comes back.
const RESET_DISCONNECT_DELAY: Duration = Duration::from_millis(15);

/// Lead time added on top of the measured `get_clock` round trip.
///
/// The round trip only shows the wire's latency at one instant: this covers the
/// host's local handling between taking the sample and handing
/// `reset_step_clock` to the link, plus jitter the sample did not capture.
const ANCHOR_SLACK: Duration = Duration::from_millis(1);

/// The anchor always leads by at least this much — exactly the fixed lead the
/// tool used before it measured the round trip, so a tiny RTT can never make
/// the margin worse than it was.
const ANCHOR_MIN_MARGIN: Duration = Duration::from_millis(1);

/// How far the stepper's clock is put ahead of now on each re-anchor, derived
/// from the measured `get_clock` round trip.
///
/// The reset travels to the firmware after the clock was read, so anchoring at
/// "now" would already be in the past and could shut the firmware down with
/// `Timer too close` before the ramp even starts. From the sampling instant to
/// `reset_step_clock` reaching the firmware is about one round trip, and the
/// measured RTT itself is one more round trip the lead has to absorb, hence
/// `2 × rtt`, plus [`ANCHOR_SLACK`] for local handling and jitter. The
/// [`ANCHOR_MIN_MARGIN`] floor keeps an RTT too small to measure from scoring
/// worse than the old fixed 1 ms margin.
fn anchor_margin(rtt: Duration) -> Duration {
    rtt.checked_mul(2)
        .unwrap_or(Duration::MAX)
        .saturating_add(ANCHOR_SLACK)
        .max(ANCHOR_MIN_MARGIN)
}

#[derive(Args, Debug)]
pub struct StressArgs {
    /// Klipper config file to read the MCU and a stepper from
    pub config_file: String,

    /// Name of the MCU(s) to stress: the sub of `[mcu <name>]`, repeated (or
    /// given several at once) to drive several boards concurrently; empty or
    /// omitted means the bare `[mcu]` section — the single default board
    #[arg(num_args = 1..)]
    pub mcu: Vec<String>,

    /// Stress every `[mcu]` section in the config instead of naming boards
    /// (the bare `[mcu]` counts as one); cannot be combined with MCU names
    #[arg(long)]
    pub all_mcus: bool,

    /// Each stage is this many times the previous one; smaller brackets the
    /// limit more tightly and takes longer
    #[arg(long, default_value_t = RATE_STEP)]
    pub rate_step: f64,

    /// How long each stage drives the stepper, in seconds
    #[arg(long, default_value_t = STAGE_SECONDS)]
    pub stage_seconds: f64,

    /// What to stress
    #[arg(long, value_enum, default_value_t = Task::Step)]
    pub task: Task,
}

/// Entry point for the `stress` subcommand.
pub fn run(args: StressArgs) -> Result<(), Box<dyn std::error::Error>> {
    // Two workers on purpose: one reactor plus the boards' send/receive tasks
    // fit in two, the number a single-board run has always had, so one- and
    // multi-board measurements stay comparable. Port I/O itself runs on the
    // blocking pool, so a second board adds tasks rather than threads — but if
    // a concurrent run ever shows the ramps competing for these workers
    // (scheduler latency in the pacing), raise it then; keep the value as it is
    // while the comparison is what matters.
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    runtime.block_on(stress(args))
}

async fn stress(args: StressArgs) -> Result<(), Box<dyn std::error::Error>> {
    if !args.rate_step.is_finite() || args.rate_step <= 1.0 {
        return Err("--rate-step must be greater than 1".into());
    }
    if !args.stage_seconds.is_finite() || args.stage_seconds <= 0.0 {
        return Err("--stage-seconds must be positive".into());
    }

    let (config, _sources) = Config::from_file(&args.config_file)?;
    let names = select_mcus(&config, &args.mcu, args.all_mcus)
        .map_err(|err| format!("{err} in {}", args.config_file))?;

    // The run's pin vocabulary: one chip per `[mcu]` section (so `[board_pins]`
    // may name any of them) and the config's aliases, loaded through the same
    // `[board_pins]` loader production uses. Every stepper below is built
    // through this registry, so its pins are parsed, aliased and numbered
    // exactly as a real load would do — and the firmware `config_stepper` gets
    // the same `invert_step`/`step_pulse_ticks` production would send.
    let pins = build_pin_registry(&config)?;

    // Connect one board at a time: each opens its own port and runs its own
    // identify, and a board that refuses must not stop the others from being
    // driven — its failure is reported at the end instead.
    let mut boards: Vec<(McuConfig, Arc<Mcu>)> = Vec::new();
    let mut failures: Vec<String> = Vec::new();
    for name in &names {
        let connected = match resolve_mcu(&config, name) {
            Ok(mcu_config) => match connect(&mcu_config).await {
                Ok(mcu) => Ok((mcu_config, mcu)),
                Err(err) => Err(err.to_string()),
            },
            Err(err) => Err(err),
        };
        match connected {
            Ok((mcu_config, mcu)) => {
                println!(
                    "{}",
                    tagged(
                        name,
                        format!(
                            "connected to MCU '{}' ({} command(s) in its dictionary)",
                            mcu_config.name,
                            mcu.dictionary().map(|d| d.commands().len()).unwrap_or(0)
                        )
                    )
                );
                boards.push((mcu_config, mcu));
            }
            Err(err) => {
                println!("{}", tagged(name, format!("connect failed: {err}")));
                failures.push(format!("{name}: {err}"));
            }
        }
    }

    // Any board's own result — the run itself, reported per board inside.
    let (run_failures, ran) = drive_boards(boards, &config, &args, &pins).await;
    failures.extend(run_failures);

    // Any board's hard error fails the command (exit code 1); a board that
    // found its limit reported that as its own result and is a success — and a
    // step run in which every board was skipped tested nothing, which fails too.
    match final_error(&failures, names.len(), ran, args.task) {
        Some(err) => Err(err.into()),
        None => Ok(()),
    }
}

/// What one board did in this run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoardOutcome {
    /// The board ran the task the command asked for.
    Ran,
    /// `--task step` skipped the board: it owns no stepper section, and only a
    /// board `--all-mcus` enumerated may be skipped this way.
    Skipped,
}

/// Every connected board's run: hand the sessions to their futures, poll them
/// together, and report each board's outcome.
///
/// The connections **move** in and nothing outside keeps a reference: this run
/// is each session's only owner, and it holds that ownership across the join.
/// [`configure_steppers`]' `drop(mcu)` in a `ResetRequired` reconnect is what
/// closes the old session before the port is reopened — a caller-side
/// `Arc<Mcu>` clone would leave it releasing nothing, and the old receive task
/// would stay a second reader on the port throughout `reconnect()`, dropping
/// the new session's frames against its own stale sequence state until
/// identify times out.
///
/// Returns the board failures (part of the command's closing error) and how
/// many boards ran their task.
async fn drive_boards(
    boards: Vec<(McuConfig, Arc<Mcu>)>,
    config: &Config,
    args: &StressArgs,
    pins: &Arc<PrinterPins>,
) -> (Vec<String>, usize) {
    let named = !args.all_mcus;
    let (mcu_configs, connections): (Vec<McuConfig>, Vec<Arc<Mcu>>) = boards.into_iter().unzip();
    // One future per connected board, polled together: the ramps interleave on
    // the same runtime, each with its own port, tasks and state. `named` says
    // which boards the command promised a stepper: every board except the ones
    // `--all-mcus` enumerated (that flag takes names of none of its own). The
    // sessions move into these futures — nothing outside the join below holds
    // one, or the reconnect's `drop(mcu)` would close nothing.
    let ramps = connections
        .into_iter()
        .zip(&mcu_configs)
        .map(|(mcu, mcu_config)| run_board(mcu_config, mcu, args.task, config, args, named, pins))
        .collect::<Vec<_>>();
    let mut failures = Vec::new();
    let mut ran = 0usize;
    for (mcu_config, outcome) in mcu_configs.iter().zip(join_all(ramps).await) {
        match outcome {
            Ok(BoardOutcome::Ran) => ran += 1,
            // The skip line is the board's whole report: it neither ran nor failed.
            Ok(BoardOutcome::Skipped) => {}
            Err(err) => {
                let err = err.to_string();
                println!("{}", tagged(&mcu_config.name, format!("failed: {err}")));
                failures.push(format!("{}: {err}", mcu_config.name));
            }
        }
    }
    (failures, ran)
}

/// One board's share of a run: the task the command asked for, as its own future.
///
/// The task functions keep their single-board signatures — this dispatcher is
/// what a concurrent run joins, one call per connected board.
async fn run_board(
    mcu_config: &McuConfig,
    mcu: Arc<Mcu>,
    task: Task,
    config: &Config,
    args: &StressArgs,
    named: bool,
    pins: &Arc<PrinterPins>,
) -> Result<BoardOutcome, Box<dyn std::error::Error>> {
    // The session's only reference from the moment this future runs: a caller
    // holding a clone would leave `configure_steppers`' `drop(mcu)` in the
    // `ResetRequired` reconnect releasing nothing, so the old receive task
    // would outlive the reopened port as a second reader on it.
    assert_eq!(
        Arc::strong_count(&mcu),
        1,
        "board '{}': the run must own its session alone, a clone would keep the old receive task alive",
        mcu_config.name
    );
    match task {
        Task::Step => step_stress(mcu_config, mcu, config, args, named, pins).await,
        Task::Comm => comm_stress(mcu, args).await.map(|()| BoardOutcome::Ran),
        Task::Motion => motion_smoke(mcu_config, mcu, config, pins)
            .await
            .map(|()| BoardOutcome::Ran),
    }
}

/// The command's closing error: the boards that failed, or a step task no
/// selected board could drive.
///
/// `selected` is how many boards the run chose, `ran` how many actually ran
/// their task. `--all-mcus` skips boards that own no stepper section; a run in
/// which every board was skipped has tested nothing, so it reports that instead
/// of exiting 0 in silence.
fn final_error(failures: &[String], selected: usize, ran: usize, task: Task) -> Option<String> {
    if !failures.is_empty() {
        return Some(format!(
            "{} of {selected} board(s) failed: {}",
            failures.len(),
            failures.join("; ")
        ));
    }
    if task == Task::Step && ran == 0 {
        return Some(
            "no selected MCU has a [stepper_*] or [manual_stepper] section; the step task had nothing to drive"
                .to_string(),
        );
    }
    None
}

/// Poll every future in one task and collect their results, in input order.
///
/// `tokio::join!` spells this combinator for a number of futures fixed at
/// compile time; how many MCUs a run has is only known at run time, so the same
/// shape is written out here with `poll_fn`: each child is polled with the
/// outer waker, so a board waiting on its port wakes the run exactly once.
fn join_all<F: Future>(futures: Vec<F>) -> impl Future<Output = Vec<F::Output>> {
    // Pinned once, here, so every child can be polled in place from the outer
    // waker without the combinator itself needing to be pinned.
    let mut pending: Vec<Option<Pin<Box<F>>>> =
        futures.into_iter().map(|f| Some(Box::pin(f))).collect();
    let mut done: Vec<Option<F::Output>> = pending.iter().map(|_| None).collect();
    poll_fn(move |cx| {
        let mut waiting = 0;
        for (slot, outcome) in pending.iter_mut().zip(done.iter_mut()) {
            if let Some(future) = slot.as_mut() {
                let polled = future.as_mut().poll(cx);
                if let Poll::Ready(value) = polled {
                    *slot = None;
                    *outcome = Some(value);
                } else {
                    waiting += 1;
                }
            }
        }
        if waiting == 0 {
            Poll::Ready(take(&mut done).into_iter().map(Option::unwrap).collect())
        } else {
            Poll::Pending
        }
    })
}

/// Prefix a report line with the board it belongs to.
///
/// Concurrent boards print as they go, so a bare line could be any board's:
/// `[zboard] …` keeps every progress and result line attributable (and greppable
/// per board).
fn tagged(mcu: &str, text: impl std::fmt::Display) -> String {
    format!("[{mcu}] {text}")
}

/// The board's closing summary: `[<mcu>] last rate it <kind>: <rate>`, or
/// `none` when the very first stage was already the limit.
///
/// This is what every ramp leaves behind on its way out — `survived` for the
/// step ramp, `carried` for the comm ramp — so a multi-board run's output ends
/// with one such line per board, each under its own prefix.
fn summary_line(mcu: &str, kind: &str, last_good: Option<f64>, unit: &str) -> String {
    let rate = last_good
        .map(|rate| format!("{rate:.0} {unit}"))
        .unwrap_or_else(|| "none".to_string());
    tagged(mcu, format!("last rate it {kind}: {rate}"))
}

/// Ramp the step rate until the MCU's step timer gives out.
///
/// The board drives **every** stepper section it owns: each gets its own oid
/// and anchor, and one ramp's pacing is applied to all of them at once — a
/// firmware stepping several steppers at once does several times the work,
/// which is the point. A board the command named that owns none fails (its
/// stepper was promised); an `--all-mcus` board that owns none is skipped.
async fn step_stress(
    mcu_config: &McuConfig,
    mcu: Arc<Mcu>,
    config: &Config,
    args: &StressArgs,
    named: bool,
    pins: &Arc<PrinterPins>,
) -> Result<BoardOutcome, Box<dyn std::error::Error>> {
    let (steppers, rejected) = find_steppers(config, &mcu_config.name, &mcu, pins);
    for reason in &rejected {
        println!("{}", tagged(&mcu_config.name, format!("skipped {reason}")));
    }
    let steppers = match plan_stepper_task(&mcu_config.name, steppers, &rejected, named) {
        StepPlan::Drive(steppers) => steppers,
        StepPlan::Skip(line) => {
            println!("{}", tagged(&mcu_config.name, line));
            return Ok(BoardOutcome::Skipped);
        }
        StepPlan::Fail(line) => return Err(line.into()),
    };
    for section in &steppers {
        let (step_pin, dir_pin) = section.numbers;
        println!(
            "{}",
            tagged(
                &mcu_config.name,
                format!(
                    "using [{}] -> step_pin={step_pin} dir_pin={dir_pin}",
                    section.name
                )
            )
        );
    }

    // The handshake can reconnect (a firmware with no `config_reset` reboots), so
    // bind the events only once the connection is final.
    let (oids, mcu) = configure_steppers(mcu_config, mcu, pins, &steppers).await?;
    let shutdown_reason: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    bind_shutdown(&mcu, &shutdown_reason)?;

    let freq = mcu.clock_freq().map_err(std::io::Error::other)?;
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!("step clock {freq:.0} Hz; ramping step rate:")
        )
    );

    // The ramp is ascending and stops at the first failure: once the firmware has
    // shut down, only a `reset` clears it, so there is no going back to a lower
    // rate. The answer is the bracket between the last rate that survived and the
    // first that did not.
    let mut rate = START_RATE;
    let mut last_good: Option<f64> = None;
    while rate <= MAX_RATE {
        let stage = stage_for(rate, freq, args.stage_seconds);
        let actual_rate = freq / f64::from(stage.interval);

        // The firmware refuses `reset_step_clock` while a move is loaded
        // (`Can't reset time when stepper active`, `src/stepper.c:311`), which
        // would otherwise look like a rate failure. Every stepper of the board
        // therefore stops, then is re-anchored, before the stage goes out: one
        // ramp, driven onto all of them at once.
        for result in join_all(
            oids.iter()
                .map(|&oid| wait_for_idle(&mcu, oid))
                .collect::<Vec<_>>(),
        )
        .await
        {
            result?;
        }
        for result in join_all(
            oids.iter()
                .map(|&oid| anchor_stepper(&mcu, oid))
                .collect::<Vec<_>>(),
        )
        .await
        {
            result?;
        }
        queue_stage(&mcu, &oids, &stage).await?;

        let wait = Duration::from_micros(stage.duration_us)
            + Duration::from_millis(20)
            + Duration::from_micros(stage.duration_us / 10);
        let steppers_note = if oids.len() > 1 {
            format!("; {} steppers", oids.len())
        } else {
            String::new()
        };
        println!(
            "{}",
            tagged(
                &mcu_config.name,
                format!(
                    "  {:>9.0} steps/s (interval {} ticks, {}x{} = {} steps over {:.0} ms{steppers_note}): queued",
                    actual_rate,
                    stage.interval,
                    stage.commands,
                    stage.count,
                    stage.steps,
                    stage.duration_us as f64 / 1000.0
                )
            )
        );
        // Let the stage run out, waking early if the firmware stops.
        sleep_until(wait, &shutdown_reason).await;

        match mcu
            .call_msg::<_, ConfigState>(&GetConfig, CALL_TIMEOUT)
            .await
        {
            Ok(state) if state.is_shutdown => {
                println!(
                    "{}",
                    tagged(
                        &mcu_config.name,
                        format!(
                            "  firmware SHUT DOWN at {actual_rate:.0} steps/s: {}",
                            shutdown_message(&shutdown_reason)
                        )
                    )
                );
                println!(
                    "{}",
                    summary_line(&mcu_config.name, "survived", last_good, "steps/s")
                );
                return Ok(BoardOutcome::Ran);
            }
            Ok(_) => {
                last_good = Some(actual_rate);
            }
            Err(err) => {
                println!(
                    "{}",
                    tagged(
                        &mcu_config.name,
                        format!(
                            "  no answer at {actual_rate:.0} steps/s ({err}); treating it as the limit"
                        )
                    )
                );
                // Every way the ramp ends leaves one summary line behind.
                println!(
                    "{}",
                    summary_line(&mcu_config.name, "survived", last_good, "steps/s")
                );
                return Ok(BoardOutcome::Ran);
            }
        }

        rate *= args.rate_step;
    }

    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!(
                "no failure up to {MAX_RATE:.0} steps/s (the ramp's top); the MCU survived every stage"
            )
        )
    );
    Ok(BoardOutcome::Ran)
}

/// Ramp the request rate on `get_clock` until the host↔MCU link gives out.
///
/// `get_clock` is a base command (it works even before `finalize_config`), so
/// this task does not configure the firmware at all — it measures the transport,
/// not the machine. The request name is fixed (`clock`), so the host cannot have
/// several in flight through `call`; the requests are fire-and-forget and the
/// answers are counted by a bound callback.
async fn comm_stress(mcu: Arc<Mcu>, args: &StressArgs) -> Result<(), Box<dyn std::error::Error>> {
    let shutdown_reason: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    bind_shutdown(&mcu, &shutdown_reason)?;

    let answered = Arc::new(AtomicU64::new(0));
    {
        let answered = Arc::clone(&answered);
        mcu.bind_callback("clock", move |_| {
            answered.fetch_add(1, Ordering::Relaxed);
        })
        .map_err(|err| std::io::Error::other(format!("bind clock: {err}")))?;
    }

    println!(
        "{}",
        tagged(
            mcu.name(),
            "ramping request rate (`get_clock` round-trips):"
        )
    );
    let mut rate = COMM_START_RATE;
    let mut last_good: Option<f64> = None;
    while rate <= COMM_MAX_RATE {
        // Let anything the last stage left in flight settle first.
        settle(&mcu).await?;
        let before = answered.load(Ordering::Relaxed);

        let sent = drive_requests(&mcu, rate, args.stage_seconds, &shutdown_reason).await?;
        let achieved = sent as f64 / args.stage_seconds;

        // Give the answers a moment to come back before measuring the backlog.
        tokio::time::sleep(Duration::from_millis(50)).await;
        let replied = answered.load(Ordering::Relaxed) - before;
        let backlog = sent.saturating_sub(replied);

        println!(
            "{}",
            tagged(
                mcu.name(),
                format!(
                    "  {:>8.0} req/s: sent {sent}, answered {replied}, achieved {achieved:.0} req/s, backlog {backlog}",
                    rate
                )
            )
        );

        let failure = if shutdown_reason
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some()
        {
            Some("the firmware shut down".to_string())
        } else if backlog > COMM_BACKLOG_LIMIT {
            Some(format!("{backlog} requests went unanswered"))
        } else if achieved < rate * 0.95 {
            Some(format!(
                "only {achieved:.0} of the {rate:.0} requests/s went out"
            ))
        } else {
            None
        };

        if let Some(failure) = failure {
            println!(
                "{}",
                tagged(
                    mcu.name(),
                    format!("  link gave out at {rate:.0} req/s: {failure}")
                )
            );
            println!(
                "{}",
                summary_line(mcu.name(), "carried", last_good, "req/s")
            );
            return Ok(());
        }
        last_good = Some(achieved);
        rate *= args.rate_step;
    }

    println!(
        "{}",
        tagged(
            mcu.name(),
            format!(
                "no failure up to {COMM_MAX_RATE:.0} req/s (the ramp's top); the link carried every stage"
            )
        )
    );
    Ok(())
}

/// Wait for anything still in flight to come back.
async fn settle(mcu: &Arc<Mcu>) -> Result<(), std::io::Error> {
    mcu.flush(CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("flush: {err}")))?;
    tokio::time::sleep(Duration::from_millis(50)).await;
    Ok(())
}

/// Send `get_clock` at `rate` requests per second for `seconds` seconds, and
/// return how many went out.
///
/// Pacing is by wall clock: each pass sends whatever is due by now and sleeps a
/// little when caught up. When the outbound channel is full (512 items,
/// `mcu/mod.rs`) the send reports it, so the pass flushes and retries once; if
/// that cannot keep up, `sent` falls behind `rate` and the caller sees it.
async fn drive_requests(
    mcu: &Arc<Mcu>,
    rate: f64,
    seconds: f64,
    shutdown: &Arc<Mutex<Option<String>>>,
) -> Result<u64, std::io::Error> {
    let start = Instant::now();
    let mut sent: u64 = 0;
    loop {
        if shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some()
        {
            break;
        }
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= seconds {
            break;
        }
        if sent < (rate * elapsed) as u64 {
            match mcu.send_msg(&GetClock) {
                Ok(()) => sent += 1,
                Err(_) => {
                    // The outbound channel is full: let it drain, then try once
                    // more. A second failure is the link giving out.
                    mcu.flush(CALL_TIMEOUT)
                        .await
                        .map_err(|err| std::io::Error::other(format!("flush: {err}")))?;
                    mcu.send_msg(&GetClock).map_err(|err| {
                        std::io::Error::other(format!("get_clock after a flush: {err}"))
                    })?;
                    sent += 1;
                }
            }
        } else {
            tokio::time::sleep(Duration::from_micros(500)).await;
        }
    }
    mcu.flush(CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("flush: {err}")))?;
    Ok(sent)
}

/// The recorded shutdown reason, or a placeholder when the firmware did not say.
fn shutdown_message(slot: &Arc<Mutex<Option<String>>>) -> String {
    slot.lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .clone()
        .unwrap_or_else(|| "(the firmware did not say why)".to_string())
}

/// One rung of the ramp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Stage {
    /// Ticks between steps.
    interval: u32,
    /// Steps in each `queue_step` command.
    count: u16,
    /// How many `queue_step` commands to send.
    commands: u32,
    /// Total steps in the stage (`commands * count`).
    steps: u64,
    /// Stage length in microseconds, as an integer so it is exactly testable.
    duration_us: u64,
}

/// Work out the interval and command mix for a target rate at `freq` ticks per
/// second.
///
/// The interval is clamped to at least one tick: a rate above the clock would
/// otherwise divide to zero, and the firmware would schedule the next step in the
/// past (`Stepper too far in past`). Each command covers one [`SLICE_SECONDS`]
/// slice, and the stage is filled with whole commands up to [`MOVE_SLOTS`], so it
/// lasts about [`STAGE_SECONDS`] at every rate.
fn stage_for(rate: f64, freq: f64, stage_seconds: f64) -> Stage {
    let interval = ((freq / rate).round() as u64).clamp(1, u64::from(u32::MAX)) as u32;
    let actual_rate = freq / f64::from(interval);
    // One time slice per command, capped by the wire type (`count=%hu`).
    let count = (actual_rate * SLICE_SECONDS)
        .round()
        .clamp(1.0, f64::from(u16::MAX)) as u16;
    // Whole commands to fill the stage, capped so one stage cannot overrun the
    // move queue.
    let ticks_per_command = u64::from(count) * u64::from(interval);
    let commands =
        ((stage_seconds * freq / ticks_per_command as f64).round() as u32).clamp(1, MOVE_SLOTS);
    let steps = u64::from(commands) * u64::from(count);
    let duration_us = (steps as f64 * f64::from(interval) / freq * 1e6).round() as u64;
    Stage {
        interval,
        count,
        commands,
        steps,
        duration_us,
    }
}

/// The name to look for: the argument, or `mcu` when it is empty.
///
/// An empty `MCU` (or omitting it) means the bare `[mcu]` section, whose name is
/// `mcu`.
fn mcu_name(arg: &str) -> &str {
    let arg = arg.trim();
    if arg.is_empty() {
        "mcu"
    } else {
        arg
    }
}

/// Find the `[mcu]` / `[mcu <name>]` section the command names.
///
/// The name is the section's `sub`, or `mcu` when there is none — the same rule
/// `McuObject::new` uses.
fn find_mcu_section<'a>(config: &'a Config, name: &str) -> Option<&'a ConfigSection> {
    config
        .sections()
        .find(|section| section.id == "mcu" && section.sub.as_deref().unwrap_or("mcu") == name)
}

/// The MCU names this run drives: the command line's list, `--all-mcus`'
/// enumeration, or the single default `mcu`.
///
/// An explicit list keeps the old spelling: an empty name means the bare
/// `[mcu]` (via [`mcu_name`]), and a board named twice is connected once.
/// `--all-mcus` takes every `[mcu]` section in config order — the bare `[mcu]`
/// among them, whose name is `mcu` — and does not take names of its own.
fn select_mcus(config: &Config, requested: &[String], all: bool) -> Result<Vec<String>, String> {
    if all {
        if !requested.is_empty() {
            return Err(
                "--all-mcus names every [mcu] section itself; it does not take MCU names".into(),
            );
        }
        let names = config
            .sections()
            .filter(|section| section.id == "mcu")
            .map(|section| section.sub.as_deref().unwrap_or("mcu").to_string())
            .collect::<Vec<_>>();
        if names.is_empty() {
            return Err("no [mcu] section".into());
        }
        return Ok(names);
    }
    if requested.is_empty() {
        return Ok(vec![mcu_name("").to_string()]);
    }
    let mut names: Vec<String> = Vec::new();
    for name in requested {
        let name = mcu_name(name).to_string();
        if !names.contains(&name) {
            names.push(name);
        }
    }
    Ok(names)
}

/// Build the configuration of the `[mcu]` / `[mcu <name>]` section one name
/// selects: the lookup [`stress`] used to do inline, plus the transport parse.
///
/// The error says which section is missing; the caller adds where it looked
/// (the config file).
fn resolve_mcu(config: &Config, name: &str) -> Result<McuConfig, String> {
    let section = find_mcu_section(config, name).ok_or_else(|| {
        if name == "mcu" {
            "no [mcu] section".to_string()
        } else {
            format!("no [mcu {name}] section")
        }
    })?;
    McuConfig::new(&ConfigWrapper::new(section, AccessTracking::shared()))
        .map_err(|err| err.to_string())
}

/// The `--task motion` smoke: drive the board's first stepper through the host's
/// motion path and read its position back.
///
/// This is the FW5f real-board smoke test: a [`Trapq`] move is solved by
/// `itersolve`, compressed by the full `stepcompress`, sent as `queue_step`
/// commands, and the firmware's own `stepper_get_position` is compared with the
/// distance the move asked for. It borrows the same step/dir pins as
/// `--task step` — and the same step distance the section configures
/// (`rotation_distance` over `microsteps` and gearing) — so it needs no full
/// `[printer]` config (and never touches the unknown Y/Z pins). The move itself
/// is [`move_one_axis`].
async fn motion_smoke(
    mcu_config: &McuConfig,
    mcu: Arc<Mcu>,
    config: &Config,
    pins: &Arc<PrinterPins>,
) -> Result<(), Box<dyn std::error::Error>> {
    /// The move the smoke asks for: `DISTANCE` mm at `SPEED` mm/s.
    const DISTANCE: f64 = 5.0;
    const SPEED: f64 = 10.0;

    // The smoke test drives one stepper: it takes the first the board owns
    // (the step task drives all of them) and reports what it rejected, if any.
    let (steppers, rejected) = find_steppers(config, &mcu_config.name, &mcu, pins);
    for reason in &rejected {
        println!("{}", tagged(&mcu_config.name, format!("skipped {reason}")));
    }
    let Some(section) = steppers.into_iter().next() else {
        return Err(no_stepper_error(&mcu_config.name, &rejected).into());
    };
    move_one_axis(mcu_config, mcu, pins, section, DISTANCE, SPEED).await
}

/// Drive one `section` through the host's motion path and read its position
/// back.
///
/// The body of [`motion_smoke`] — a single [`Stepper::cartesian`] move of
/// `distance` mm at `speed` mm/s, solved by `itersolve`, compressed by the full
/// `stepcompress`, sent as `queue_step`, and checked against the firmware's own
/// `stepper_get_position`. The smoke names the board's first stepper; the R5
/// hardware case names its own, so this takes the section to drive.
async fn move_one_axis(
    mcu_config: &McuConfig,
    mcu: Arc<Mcu>,
    pins: &Arc<PrinterPins>,
    section: StepperSource,
    distance: f64,
    speed: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    /// How long to wait for the firmware to report the expected position.
    const SETTLE_TIMEOUT: Duration = Duration::from_secs(5);
    const SETTLE_POLL: Duration = Duration::from_millis(5);

    let (step_pin, dir_pin) = section.numbers;
    let step_dist = section.options.step_dist.unwrap_or(DEFAULT_SMOKE_STEP_DIST);
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!(
                "using [{}] -> step_pin={step_pin} dir_pin={dir_pin} (step_dist={step_dist:.6} mm)",
                section.name
            )
        )
    );
    let (oids, mcu) =
        configure_steppers(mcu_config, mcu, pins, std::slice::from_ref(&section)).await?;
    let oid = oids[0];
    let freq = mcu.clock_freq().map_err(std::io::Error::other)?;

    // The host's print time *is* absolute board time (the compressor maps print
    // time zero to clock zero), so the move has to start from the board's
    // current clock, not from a fixed number. Seed the estimate from `get_uptime`.
    if mcu.has_message(GetUptime::NAME) {
        if let Ok(uptime) = mcu
            .call_msg::<GetUptime, Uptime>(&GetUptime, CALL_TIMEOUT)
            .await
        {
            mcu.set_clock_base(uptime.clock64());
        }
    }
    let now = mcu
        .estimated_clock()
        .map(|clock| clock as f64 / freq)
        .unwrap_or(1.0);

    let mut stepper = Stepper::cartesian("smoke", u32::from(oid), step_dist, Axis::X, freq);
    // Start 100 ms after the board's now, so the first step is comfortably in the
    // future and the whole schedule lands where the firmware expects it.
    let print_time = now + 0.1;
    let duration = distance / speed;
    let mut trapq = Trapq::new();
    trapq.append(
        print_time,
        0.0,
        duration,
        0.0,
        Xyz::default(),
        Xyz::new(1.0, 0.0, 0.0),
        speed,
        speed,
        0.0,
    );
    let flush_time = print_time + duration + 0.01;
    let commands = stepper.generate(&trapq, flush_time)?;
    let expected = (distance / step_dist).round() as i32;
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!(
                "generated {} command(s) for {expected} step(s): {}",
                commands.len(),
                describe_commands(&commands)
            )
        )
    );

    send_steps(&mcu, oid, &commands).await?;
    let position = wait_for_position(&mcu, oid, expected, SETTLE_TIMEOUT, SETTLE_POLL).await?;
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!("firmware position: {position} step(s), expected {expected}")
        )
    );
    if position != expected {
        return Err(format!("position mismatch: {position} != {expected}").into());
    }
    println!("{}", tagged(&mcu_config.name, "motion smoke OK"));
    Ok(())
}

/// Drive one `section` at a single steady step rate for a few seconds and report
/// the rate the firmware held — the R11 short soak.
///
/// Unlike [`step_stress`], which ramps until the firmware shuts down, the rate
/// here is **fixed** and chosen well under the board's limit, so the question is
/// "does a known-safe load hold", not "where is the limit". It queues
/// [`stage_for`] stages back to back at `rate` until `seconds` are covered,
/// re-anchoring between them the way the ramp does, then asks the firmware two
/// things: is it still configured (not shut down), and did its own
/// `stepper_get_position` advance by exactly the steps queued ("no lost steps").
///
/// Returns the **actual** rate (`clock_freq / interval`, which is what the
/// firmware saw). Errors — carrying the shutdown reason when there is one — if
/// the firmware shut down or a step went missing.
#[cfg(test)]
async fn step_soak(
    mcu_config: &McuConfig,
    mcu: Arc<Mcu>,
    pins: &Arc<PrinterPins>,
    section: StepperSource,
    rate: f64,
    seconds: f64,
) -> Result<f64, Box<dyn std::error::Error>> {
    let (oids, mcu) =
        configure_steppers(mcu_config, mcu, pins, std::slice::from_ref(&section)).await?;
    let oid = oids[0];
    let shutdown_reason: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    bind_shutdown(&mcu, &shutdown_reason)?;
    let freq = mcu.clock_freq().map_err(std::io::Error::other)?;

    // One stage is at most `MOVE_SLOTS` commands (`stage_for` caps it), so a
    // several-second soak repeats it; the rate is the same for every stage.
    let stage = stage_for(rate, freq, STAGE_SECONDS);
    let actual_rate = freq / f64::from(stage.interval);
    let stage_seconds = stage.duration_us as f64 / 1e6;
    let stages = (seconds / stage_seconds).ceil().max(1.0) as u32;
    let queued_per_stepper = u64::from(stages) * stage.steps;
    let travel = section.options.step_dist.map(|step_dist| {
        format!(
            " (~{:.0} mm at {step_dist:.5} mm/step)",
            queued_per_stepper as f64 * step_dist
        )
    });
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!(
                "soak: {actual_rate:.0} steps/s for {:.1} s in {stages} stage(s) -> {queued_per_stepper} step(s){}",
                stages as f64 * stage_seconds,
                travel.unwrap_or_default(),
            )
        )
    );

    // The firmware's counter is direction-signed and accumulates across moves and
    // `reset_step_clock`, so the soak measures the *change*: the axis's move, not
    // where it started.
    let start = mcu
        .call_msg::<_, StepperPosition>(&StepperGetPosition { oid }, CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("stepper_get_position: {err}")))?
        .pos;

    let mut queued: i64 = 0;
    for _ in 0..stages {
        if shutdown_reason
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some()
        {
            break;
        }
        wait_for_idle(&mcu, oid).await?;
        anchor_stepper(&mcu, oid).await?;
        queue_stage(&mcu, &oids, &stage).await?;
        queued += stage.steps as i64;
        // Let the stage run out, waking early if the firmware stops.
        let wait = Duration::from_micros(stage.duration_us)
            + Duration::from_millis(20)
            + Duration::from_micros(stage.duration_us / 10);
        sleep_until(wait, &shutdown_reason).await;
    }

    let state = mcu
        .call_msg::<_, ConfigState>(&GetConfig, CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("get_config: {err}")))?;
    if state.is_shutdown {
        return Err(format!(
            "the firmware shut down at {actual_rate:.0} steps/s: {}",
            shutdown_message(&shutdown_reason)
        )
        .into());
    }

    // "No lost steps": every step put on the wire has to have happened. The sign
    // is the firmware's direction convention; the magnitude is what matters.
    let moved = i64::from(
        mcu.call_msg::<_, StepperPosition>(&StepperGetPosition { oid }, CALL_TIMEOUT)
            .await
            .map_err(|err| std::io::Error::other(format!("stepper_get_position: {err}")))?
            .pos,
    ) - i64::from(start);
    if moved.abs() != queued {
        return Err(format!(
            "the stepper counted {moved} step(s), not the {queued} queued at {actual_rate:.0} steps/s"
        )
        .into());
    }
    Ok(actual_rate)
}

/// Send the compressor's commands, in batches small enough for the send queue.
async fn send_steps(
    mcu: &Arc<Mcu>,
    oid: u8,
    commands: &[StepCommand],
) -> Result<(), std::io::Error> {
    // The outbound channel holds 512 items (`mcu/mod.rs`); a flush every 16 keeps
    // it from filling while a long compressed run goes out.
    const BATCH: usize = 16;
    for chunk in commands.chunks(BATCH) {
        for command in chunk {
            match *command {
                StepCommand::SetNextStepDir { direction, .. } => mcu
                    .send_msg(&SetNextStepDir {
                        oid,
                        dir: u8::from(direction),
                    })
                    .map_err(|err| std::io::Error::other(format!("set_next_step_dir: {err}")))?,
                StepCommand::QueueStep {
                    interval,
                    count,
                    add,
                    ..
                } => mcu
                    .send_msg(&QueueStep {
                        oid,
                        interval,
                        count: count as u16,
                        add: add as i16,
                    })
                    .map_err(|err| std::io::Error::other(format!("queue_step: {err}")))?,
            }
        }
        mcu.flush(CALL_TIMEOUT)
            .await
            .map_err(|err| std::io::Error::other(format!("flush: {err}")))?;
    }
    Ok(())
}

/// Poll the firmware's step counter until it reaches `expected`.
async fn wait_for_position(
    mcu: &Arc<Mcu>,
    oid: u8,
    expected: i32,
    timeout: Duration,
    poll: Duration,
) -> Result<i32, std::io::Error> {
    let deadline = Instant::now() + timeout;
    let mut last = mcu
        .call_msg::<_, StepperPosition>(&StepperGetPosition { oid }, CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("stepper_get_position: {err}")))?
        .pos;
    while last != expected && Instant::now() < deadline {
        tokio::time::sleep(poll).await;
        last = mcu
            .call_msg::<_, StepperPosition>(&StepperGetPosition { oid }, CALL_TIMEOUT)
            .await
            .map_err(|err| std::io::Error::other(format!("stepper_get_position: {err}")))?
            .pos;
    }
    if last == expected {
        Ok(last)
    } else {
        Err(std::io::Error::other(format!(
            "the stepper reached {last}, not {expected}"
        )))
    }
}

/// A short summary of a command list, for the smoke test's log line.
fn describe_commands(commands: &[StepCommand]) -> String {
    let steps: u32 = commands
        .iter()
        .filter_map(|command| match command {
            StepCommand::QueueStep { count, .. } => Some(*count),
            StepCommand::SetNextStepDir { .. } => None,
        })
        .sum();
    let dirs = commands
        .iter()
        .filter(|command| matches!(command, StepCommand::SetNextStepDir { .. }))
        .count();
    format!("{steps} step(s), {dirs} dir command(s)")
}

/// Every stepper section this board drives, and the sections rejected on the way.
///
/// **Ownership is `step_pin`'s chip alone**: a pin written `chip:pin` belongs to
/// that MCU, an unprefixed pin names the primary MCU (`mcu`), so a section with
/// `step_pin: PA0` is this board's only if this *is* the primary one. No other
/// pin takes part in the choice — `enable_pin` may sit on another board and an
/// `endstop_pin` on another board does not make that stepper theirs. `dir_pin`
/// is the one exception: upstream refuses a stepper whose `dir_pin` names a
/// different MCU than its `step_pin` (`src/core/klippy/extras/stepper.rs`:
/// `Stepper dir pin must be on same mcu as step pin`), so such a section is
/// rejected here with that wording — per section, so one bad section does not
/// sink the board.
///
/// The pin descriptions are parsed by the same [`PrinterPins`] production uses,
/// so `!` (invert), `chip:` prefixes and `[board_pins]` aliases all mean here
/// what they mean there, and the step pin's `!` is carried as upstream's
/// `invert_step`. A section is either collected — its step/dir pair resolved
/// through the registry and the numbered pins named this board's — or rejected
/// with a message that starts with the section's name. A section with no
/// step/dir pair at all is skipped silently, as it always was.
fn find_steppers(
    config: &Config,
    mcu_name: &str,
    mcu: &Mcu,
    pins: &PrinterPins,
) -> (Vec<StepperSource>, Vec<String>) {
    let mut selected: Vec<StepperSource> = Vec::new();
    let mut rejected: Vec<String> = Vec::new();
    for section in config.sections() {
        if !(section.id.starts_with("stepper_") || section.id == "manual_stepper") {
            continue;
        }
        let (Some(step), Some(dir)) = (
            section_string(section, "step_pin"),
            section_string(section, "dir_pin"),
        ) else {
            continue;
        };
        let name = section.identifier();
        let step_params = match pins.parse_pin(&step, true, false) {
            Ok(params) => params,
            Err(err) => {
                rejected.push(format!("{name}: {err}"));
                continue;
            }
        };
        // Ownership: `step_pin`'s chip; an unprefixed pin names the bare `[mcu]`.
        if step_params.chip_name != mcu_name {
            continue;
        }
        let dir_params = match pins.parse_pin(&dir, true, false) {
            Ok(params) => params,
            Err(err) => {
                rejected.push(format!("{name}: {err}"));
                continue;
            }
        };
        if dir_params.chip_name != mcu_name {
            rejected.push(format!(
                "{name}: Stepper dir pin must be on same mcu as step pin (step_pin '{step}', dir_pin '{dir}')"
            ));
            continue;
        }
        let numbers = match (
            resolve_number(pins, mcu, mcu_name, &step_params),
            resolve_number(pins, mcu, mcu_name, &dir_params),
        ) {
            (Ok(step_pin), Ok(dir_pin)) => (step_pin, dir_pin),
            (Err(err), _) | (_, Err(err)) => {
                rejected.push(format!("{name}: {err}"));
                continue;
            }
        };
        let options = match read_stepper_options(section, pins, mcu, mcu_name) {
            Ok(options) => options,
            Err(err) => {
                rejected.push(format!("{name}: {err}"));
                continue;
            }
        };
        selected.push(StepperSource {
            name,
            step_pin: step,
            dir_pin: dir,
            numbers,
            options,
        });
    }
    (selected, rejected)
}

/// One stepper section a board drives: the pins as written and the options
/// production feeds into the firmware stepper.
///
/// The pins stay as written (with `!`/`chip:`/alias intact) so the real
/// resource can resolve them again when it builds `config_stepper`; `numbers`
/// is what the report line prints and what the per-section rejection was based
/// on. The options are read by [`read_stepper_options`].
#[derive(Debug, Clone)]
struct StepperSource {
    /// The section identifier (`stepper_x`, `manual_stepper docking`).
    name: String,
    /// `step_pin` as written.
    step_pin: String,
    /// `dir_pin` as written.
    dir_pin: String,
    /// The firmware numbers the two pins resolve to, for the report line.
    numbers: (u8, u8),
    /// What `[stepper_*]` gives the firmware stepper beyond the pins.
    options: StepperOptions,
}

/// The section options the stress stepper takes over from production.
#[derive(Debug, Clone)]
struct StepperOptions {
    /// `step_pulse_duration` in seconds (upstream's default when absent),
    /// which the resource turns into `step_pulse_ticks` on the wire.
    step_pulse_duration: f64,
    /// `enable_pin` as written, when the section names one: the driver is
    /// switched on for the run, as production's `stepper_enable` does.
    enable_pin: Option<String>,
    /// The chip `enable_pin` names, when that pin is this board's: only then is
    /// it driven here (another board's enable is its own run's business).
    enable_chip: Option<String>,
    /// Millimetres per step (`rotation_distance` over `microsteps` and gearing),
    /// what the `--task motion` smoke moves by. `None` when the section carries
    /// no production geometry at all (the step task needs none).
    step_dist: Option<f64>,
}

/// The pulse width upstream uses when `step_pulse_duration` is absent
/// (`klippy/stepper.py:80`, `extras/stepper.rs`'s `DEFAULT_STEP_PULSE_DURATION`).
const DEFAULT_STEP_PULSE_DURATION: f64 = 0.000_002;

/// The step distance the motion smoke moves by when the section names no
/// geometry — the value the smoke always used (100 steps/mm). A section that
/// *does* spell out geometry is held to production's rules instead.
const DEFAULT_SMOKE_STEP_DIST: f64 = 0.01;

/// Read the `[stepper_*]` / `[manual_stepper]` options the stress stepper takes
/// from production: the pulse width, the enable pin, and the step distance.
///
/// The bounds are the ones `extras/stepper.rs` applies, so a value production
/// would refuse is refused here too rather than silently clamped.
fn read_stepper_options(
    section: &ConfigSection,
    pins: &PrinterPins,
    mcu: &Mcu,
    mcu_name: &str,
) -> Result<StepperOptions, String> {
    let config = ConfigWrapper::untracked(section);
    let step_pulse_duration = config
        .get_float_bounded(
            "step_pulse_duration",
            Some(DEFAULT_STEP_PULSE_DURATION),
            Some(0.0),
            Some(0.001),
            None,
            None,
        )
        .map_err(|err| err.to_string())?;
    // The enable pin is only driven when it is this board's; a pin on another
    // board is neither validated here nor touched (that board drives its own).
    let (enable_pin, enable_chip) = match config.get_str("enable_pin") {
        Some(enable) => {
            let params = pins
                .parse_pin(&enable, true, false)
                .map_err(|err| err.to_string())?;
            // Only this board's own enable pin is checked here; one on another
            // board is left to that board's run.
            if params.chip_name == mcu_name {
                resolve_number(pins, mcu, mcu_name, &params)?;
            }
            (Some(enable), Some(params.chip_name))
        }
        None => (None, None),
    };
    let step_dist = read_step_dist(&config, section)?;
    Ok(StepperOptions {
        step_pulse_duration,
        enable_pin,
        enable_chip,
        step_dist,
    })
}

/// The step distance production would use: `rotation_distance` over full steps,
/// `microsteps` and gearing (`extras/stepper.rs`'s `parse_step_distance`),
/// including its radians inference for a section with `gear_ratio` but no
/// `rotation_distance`.
///
/// `None` when the section carries no geometry at all: the step task only
/// borrows step/dir pins, so it must keep working on a section with nothing
/// else. A section that names *some* of the geometry is held to production's
/// rules, so a half-written geometry is an error rather than a silent default.
fn read_step_dist(config: &ConfigWrapper, section: &ConfigSection) -> Result<Option<f64>, String> {
    if !config.has("rotation_distance") && !config.has("gear_ratio") && !config.has("microsteps") {
        return Ok(None);
    }
    let identifier = section.identifier();
    let microsteps = config
        .get_int_bounded("microsteps", None, Some(1), None)
        .map_err(|err| err.to_string())?;
    let full_steps = config
        .get_int("full_steps_per_rotation", Some(200))
        .map_err(|err| err.to_string())?;
    if full_steps < 1 || full_steps % 4 != 0 {
        return Err(format!(
            "full_steps_per_rotation invalid in section '{identifier}'"
        ));
    }
    let gear_ratio = config
        .get_list_of_lists("gear_ratio", ',', ':', 2)
        .map_err(|err| err.to_string())?
        .into_iter()
        .map(|pair| {
            let first = pair[0].trim().parse::<f64>().map_err(|_| {
                format!("Unable to parse option 'gear_ratio' in section '{identifier}'")
            })?;
            let second = pair[1].trim().parse::<f64>().map_err(|_| {
                format!("Unable to parse option 'gear_ratio' in section '{identifier}'")
            })?;
            if second == 0.0 {
                return Err(format!(
                    "Option 'gear_ratio' in section '{identifier}' must not divide by zero"
                ));
            }
            Ok(first / second)
        })
        .collect::<Result<Vec<f64>, String>>()?
        .into_iter()
        .product::<f64>()
        .max(f64::MIN_POSITIVE);
    let units_in_radians = !config.has("rotation_distance") && config.has("gear_ratio");
    let rotation_distance = if units_in_radians {
        std::f64::consts::TAU
    } else {
        config
            .get_float_bounded("rotation_distance", None, None, None, Some(0.0), None)
            .map_err(|err| err.to_string())?
    };
    let steps_per_rotation = full_steps as f64 * microsteps as f64 * gear_ratio;
    Ok(Some(rotation_distance / steps_per_rotation))
}

/// Resolve a parsed pin to its firmware number through the F2 registry: follow
/// aliases first, then look the canonical name up in the board's enumeration.
fn resolve_number(
    pins: &PrinterPins,
    mcu: &Mcu,
    chip_name: &str,
    params: &PinParams,
) -> Result<u8, String> {
    let canonical = pins
        .resolve_pin(chip_name, &params.pin)
        .map_err(|err| err.to_string())?;
    let number = pin_number(mcu, &canonical, chip_name).map_err(|err| err.to_string())?;
    u8::try_from(number).map_err(|_| {
        format!("pin '{canonical}' is number {number}, which does not fit the byte `config_stepper` wants")
    })
}

/// The hard error for a board that was promised a stepper and owns none.
///
/// With nothing rejected the message is the one a board without a stepper has
/// always got (which the docs quote); when sections were rejected, they say
/// why they were.
fn no_stepper_error(mcu_name: &str, rejected: &[String]) -> String {
    if rejected.is_empty() {
        format!(
            "no [stepper_*] or [manual_stepper] section on MCU '{mcu_name}' to take a step/dir pin pair from"
        )
    } else {
        format!(
            "no usable [stepper_*] or [manual_stepper] section on MCU '{mcu_name}': {}",
            rejected.join("; ")
        )
    }
}

/// What `--task step` does with one board: drive every stepper it owns, skip
/// the board, or fail it.
#[derive(Debug)]
enum StepPlan {
    /// Drive all of the board's steppers — one oid and anchor each, one ramp.
    Drive(Vec<StepperSource>),
    /// `--all-mcus` found a board with no stepper of its own: print this line
    /// and move on, without touching the exit code.
    Skip(String),
    /// The command named this board itself, so a stepper was promised: fail
    /// the board (which fails the command) with this message.
    Fail(String),
}

/// Decide a board's step-task fate from what [`find_steppers`] found.
///
/// `named` distinguishes a board the command spelled out (or the default bare
/// `[mcu]`) from one `--all-mcus` enumerated: only the latter may be skipped —
/// a named board without a stepper is the error it has always been.
fn plan_stepper_task(
    mcu_name: &str,
    steppers: Vec<StepperSource>,
    rejected: &[String],
    named: bool,
) -> StepPlan {
    if !steppers.is_empty() {
        return StepPlan::Drive(steppers);
    }
    if named {
        StepPlan::Fail(no_stepper_error(mcu_name, rejected))
    } else if rejected.is_empty() {
        StepPlan::Skip("no stepper section on this MCU; skipping step task".to_string())
    } else {
        StepPlan::Skip("no usable stepper section on this MCU; skipping step task".to_string())
    }
}

/// Open the transport and run the identify handshake.
async fn connect(mcu_config: &McuConfig) -> Result<Arc<Mcu>, std::io::Error> {
    let interface = mcu_config.open().map_err(std::io::Error::other)?;
    Mcu::connect(&mcu_config.name, interface)
        .await
        .map_err(|err| std::io::Error::other(format!("{}: {err}", mcu_config.name)))
}

/// Reopen the board after it was told to `reset`, retrying while it re-enumerates.
async fn reconnect(mcu_config: &McuConfig) -> Result<Arc<Mcu>, std::io::Error> {
    let mut last = String::new();
    for _ in 0..RECONNECT_ATTEMPTS {
        tokio::time::sleep(RECONNECT_DELAY).await;
        match connect(mcu_config).await {
            Ok(mcu) => return Ok(mcu),
            Err(err) => last = err.to_string(),
        }
    }
    Err(std::io::Error::other(format!(
        "MCU '{}' did not come back after a reset: {last}",
        mcu_config.name
    )))
}

/// The run's pin registry: one chip per `[mcu]` section and the config's
/// `[board_pins]` aliases, built the way a real load builds them.
///
/// Every board in the config gets a chip — not only the ones being driven —
/// because `[board_pins]` may name any of them, exactly as `pins.py`'s
/// `register_chip` walk would. Each chip owns the [`ConfigBuilder`] its
/// steppers add `config_stepper` to.
fn build_pin_registry(config: &Config) -> Result<Arc<PrinterPins>, Box<dyn std::error::Error>> {
    let pins = Arc::new(PrinterPins::new());
    for section in config.sections().filter(|section| section.id == "mcu") {
        let name = section.sub.as_deref().unwrap_or("mcu").to_string();
        let chip = McuChip::new(
            name.clone(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip(&name, Arc::new(chip))
            .map_err(|err| format!("[mcu {name}]: {err}"))?;
    }
    load_board_pins(config, &pins)?;
    Ok(pins)
}

/// Apply the config's `[board_pins]` sections to the registry through the same
/// loader production uses, so an alias means exactly what it means there.
///
/// The loader finds the `pins` object on a printer, so the registry is put on a
/// bare one for the walk: no other object is registered or read, and the
/// printer is dropped once the aliases are in.
fn load_board_pins(
    config: &Config,
    pins: &Arc<PrinterPins>,
) -> Result<(), Box<dyn std::error::Error>> {
    let printer = Arc::new(Printer::new(ManualReactor::shared()));
    printer.add_object(PINS_OBJECT, pins.clone())?;
    for section in config
        .sections()
        .filter(|section| section.id == "board_pins")
    {
        BoardPins::new(
            &ConfigWrapper::new(section, AccessTracking::shared()),
            &printer,
        )
        .map_err(|err| format!("{}: {err}", section.identifier()))?;
    }
    Ok(())
}

/// Configure the board's stress steppers: one oid and one move-queue reserve
/// each, **one** configuration handshake for the board, and the connection
/// (which a reconnect may have replaced).
///
/// A board with several steppers is configured in a single round rather than
/// one handshake per stepper: `finalize_config` locks the firmware
/// (`src/basecmd.c:173` — a second one shuts down with `Already finalized`),
/// and a later handshake carrying a different CRC would `config_reset` the
/// board and drop every stepper configured before it.
///
/// Each stepper is the **real resource** (`PrinterPins::setup_stepper`), so the
/// firmware `config_stepper` carries the section's own `invert_step` (the step
/// pin's `!`) and `step_pulse_ticks` (`step_pulse_duration`), and its pins are
/// resolved through the same registry (aliases included) production uses. The
/// section's `enable_pin`, when it names one on this board, is switched on for
/// the run as `stepper_enable` would.
///
/// A firmware with no `config_reset` can only accept the configuration by
/// rebooting itself (`ResetRequired`), which drops the connection, so this
/// follows `McuObject`: send `reset`, reconnect, retry the same built config.
async fn configure_steppers(
    mcu_config: &McuConfig,
    mut mcu: Arc<Mcu>,
    pins: &PrinterPins,
    sections: &[StepperSource],
) -> Result<(Vec<u8>, Arc<Mcu>), std::io::Error> {
    let chip = pins.chip_as::<McuChip>(&mcu_config.name).ok_or_else(|| {
        std::io::Error::other(format!(
            "no pin chip registered for MCU '{}'",
            mcu_config.name
        ))
    })?;
    let builder = chip.config();

    let mut steppers = Vec::with_capacity(sections.len());
    // Shared enable lines are written the same way on every section that uses
    // them, so the pin is switched on once.
    let mut enabled: Vec<String> = Vec::new();
    for section in sections {
        let stepper = pins
            .setup_stepper(
                &section.step_pin,
                &section.dir_pin,
                section.options.step_pulse_duration,
            )
            .map_err(|err| std::io::Error::other(format!("{}: {err}", section.name)))?;
        for _ in 0..MOVE_SLOTS {
            builder
                .request_move_queue_slot()
                .map_err(|err| std::io::Error::other(format!("request_move_queue_slot: {err}")))?;
        }
        if let Some(enable) = &section.options.enable_pin {
            if section.options.enable_chip.as_deref() == Some(&mcu_config.name)
                && !enabled.iter().any(|seen| seen == enable)
            {
                pins.setup_static_digital_out(enable).map_err(|err| {
                    std::io::Error::other(format!("{}: enable_pin: {err}", section.name))
                })?;
                enabled.push(enable.clone());
            }
        }
        steppers.push(stepper);
    }

    let mut built = builder
        .build(&mcu)
        .map_err(|err| std::io::Error::other(format!("build: {err}")))?;
    // The resource assigned its oid in its config callback, which just ran.
    let oids = steppers
        .iter()
        .map(|stepper| stepper.oid())
        .collect::<Result<Vec<u8>, McuError>>()
        .map_err(|err| std::io::Error::other(format!("stepper oid: {err}")))?;
    // A board already carrying another configuration has to reboot before this
    // one can be sent. One `reset` normally settles it; a second covers a board
    // that was reconfigured while it rebooted (a real printer host reconnecting
    // to it), and then the run gives up rather than looping — the board is
    // under someone else's control.
    const MAX_RESET_ATTEMPTS: u32 = 2;
    let mut reset_attempts = 0;
    let configured = loop {
        match builder.handshake(&mcu, &mut built, false).await {
            Ok(configured) => break configured,
            Err(McuError::ResetRequired) if reset_attempts < MAX_RESET_ATTEMPTS => {
                reset_attempts += 1;
                // A firmware that offers `reset` — every ARM board, the F103
                // included — is rebooted rather than cleared in place, so this
                // is the path a board already carrying another configuration
                // takes (`reset_firmware` in `mcu/config.rs` prefers `reset`).
                println!(
                    "{}",
                    tagged(
                        &mcu_config.name,
                        format!(
                            "board already carries a different configuration; sending 'reset' and reconnecting (attempt {reset_attempts}/{MAX_RESET_ATTEMPTS})"
                        )
                    )
                );
                let reset_started = Instant::now();
                mcu.send_msg(&Reset)
                    .map_err(|err| std::io::Error::other(format!("reset: {err}")))?;
                let _ = mcu.flush(RESET_FLUSH_TIMEOUT).await;
                // The old session's receive task still owns the port: left
                // alive it would be a second reader on the same tty throughout
                // `reconnect()`, stealing the new session's frames and dropping
                // them against its own (stale) sequence state — the new session
                // then starves until identify times out. Give the reset bytes
                // the drain pause, then drop the old `Mcu` first (its `Drop`
                // shuts the interface down and aborts the receive task) and
                // only then reopen, as upstream does between `reset` and the
                // reconnect (`klippy/mcu.py`: pause, `_disconnect()`, reopen).
                tokio::time::sleep(RESET_DISCONNECT_DELAY).await;
                // The same requirement, checked where it matters: this `drop`
                // closes the session only when nothing else holds it. A
                // surviving clone turns it into a no-op and leaves the old
                // receive task stealing the new session's frames (vA: hundreds
                // of `Frame with sequence … dropping it` across every reopen).
                assert_eq!(
                    Arc::strong_count(&mcu),
                    1,
                    "MCU '{}': the old session is still referenced; dropping it would leave its receive task on the port",
                    mcu_config.name
                );
                drop(mcu);
                mcu = reconnect(mcu_config).await?;
                println!(
                    "{}",
                    tagged(
                        &mcu_config.name,
                        format!(
                            "reconnected {:.2} s after the reset; retrying the configuration handshake",
                            reset_started.elapsed().as_secs_f64()
                        )
                    )
                );
            }
            Err(McuError::ResetRequired) => {
                return Err(std::io::Error::other(format!(
                    "MCU '{}' still carries a configuration after {reset_attempts} 'reset' attempt(s); is a real printer host holding the board?",
                    mcu_config.name
                )));
            }
            Err(err) => {
                return Err(std::io::Error::other(format!("configure: {err}")));
            }
        }
    };
    let described = if oids.len() == 1 {
        format!("stepper oid {}", oids[0])
    } else {
        format!(
            "{} steppers (oids {})",
            oids.len(),
            oids.iter()
                .map(u8::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        )
    };
    println!(
        "{}",
        tagged(
            &mcu_config.name,
            format!(
                "configured {described} (firmware move queue: {} slots{})",
                configured.move_count,
                if configured.reused {
                    ", config reused"
                } else {
                    ""
                }
            )
        )
    );
    Ok((oids, mcu))
}

/// One stage's `queue_step` commands, in the order they go out.
///
/// Every stepper of the board gets `stage.commands` commands carrying the same
/// `interval`/`count` schedule — one ramp, driven onto all of them at once —
/// laid out round by round, so a batch spans the steppers instead of one
/// stepper's whole stage.
fn stage_commands(oids: &[u8], stage: &Stage) -> Vec<QueueStep> {
    let mut commands = Vec::new();
    let mut queued = vec![0u32; oids.len()];
    while queued.iter().any(|sent| *sent < stage.commands) {
        for (oid, sent) in oids.iter().zip(queued.iter_mut()) {
            let batch = (stage.commands - *sent).min(SEND_BATCH);
            for _ in 0..batch {
                commands.push(QueueStep {
                    oid: *oid,
                    interval: stage.interval,
                    count: stage.count,
                    add: 0,
                });
            }
            *sent += batch;
        }
    }
    commands
}

/// Queue one stage's commands onto every stepper of the board, in `SEND_BATCH`
/// batches with a `flush` between them — the outbound channel holds 512 items
/// (`mcu/mod.rs`), and batching by command count keeps it from filling however
/// many steppers share the board. The flush paces the *sending*, not the
/// stepping: the commands chain, so the schedule the firmware sees is the one
/// [`stage_commands`] laid out.
async fn queue_stage(mcu: &Arc<Mcu>, oids: &[u8], stage: &Stage) -> Result<(), std::io::Error> {
    let commands = stage_commands(oids, stage);
    let mut sent = 0;
    while sent < commands.len() {
        let end = (sent + SEND_BATCH as usize).min(commands.len());
        for command in &commands[sent..end] {
            mcu.send_msg(command)
                .map_err(|err| std::io::Error::other(format!("queue_step: {err}")))?;
        }
        mcu.flush(CALL_TIMEOUT)
            .await
            .map_err(|err| std::io::Error::other(format!("flush: {err}")))?;
        sent = end;
    }
    Ok(())
}

/// Record the firmware's shutdown reason from its `shutdown` / `is_shutdown`
/// events (both carry `static_string_id`, resolved through the dictionary).
///
/// The handlers run on the receive task, so a plain mutex is enough.
fn bind_shutdown(mcu: &Mcu, slot: &Arc<Mutex<Option<String>>>) -> Result<(), std::io::Error> {
    use crate::core::klippy::event::{IsShutdown, Shutdown, Stats};

    let recorded = Arc::clone(slot);
    mcu.bind_event::<Shutdown, _>(move |event| {
        *recorded.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(event.reason);
    })
    .map_err(|err| std::io::Error::other(format!("bind shutdown: {err}")))?;

    let recorded = Arc::clone(slot);
    mcu.bind_event::<IsShutdown, _>(move |event| {
        *recorded.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(event.reason);
    })
    .map_err(|err| std::io::Error::other(format!("bind is_shutdown: {err}")))?;

    // The firmware sends `stats` on its own; nothing here consumes it, and an
    // unbound message is logged as "Unhandled". Bind it to keep the ramp's output
    // clean.
    mcu.bind_event::<Stats, _>(|_| {})
        .map_err(|err| std::io::Error::other(format!("bind stats: {err}")))?;
    Ok(())
}

/// Wait until the stepper stops moving.
///
/// `reset_step_clock` shuts the firmware down if a move is still loaded
/// (`Can't reset time when stepper active`, `src/stepper.c:311`), so the stage
/// before it must be completely done. The reported position moves while a move is
/// loaded and stops when none is, so two equal readings in a row mean idle.
async fn wait_for_idle(mcu: &Arc<Mcu>, oid: u8) -> Result<i32, std::io::Error> {
    const POLL: Duration = Duration::from_millis(5);
    const TIMEOUT: Duration = Duration::from_secs(5);

    let deadline = Instant::now() + TIMEOUT;
    let mut last: Option<i32> = None;
    loop {
        let position = mcu
            .call_msg::<_, StepperPosition>(&StepperGetPosition { oid }, CALL_TIMEOUT)
            .await
            .map_err(|err| std::io::Error::other(format!("stepper_get_position: {err}")))?
            .pos;
        if last == Some(position) {
            return Ok(position);
        }
        last = Some(position);
        if Instant::now() >= deadline {
            return Err(std::io::Error::other(
                "the stepper never went idle after a stage; it cannot keep up",
            ));
        }
        tokio::time::sleep(POLL).await;
    }
}

/// Sleep for `wait`, waking early if the firmware reports a shutdown.
async fn sleep_until(wait: Duration, shutdown: &Arc<Mutex<Option<String>>>) {
    let deadline = Instant::now() + wait;
    loop {
        if shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some()
        {
            return;
        }
        let now = Instant::now();
        if now >= deadline {
            return;
        }
        tokio::time::sleep((deadline - now).min(Duration::from_millis(20))).await;
    }
}

/// Re-anchor the stepper's clock just ahead of now, so the next burst's first
/// step is in the future.
async fn anchor_stepper(mcu: &Arc<Mcu>, oid: u8) -> Result<(), std::io::Error> {
    let sampled_at = Instant::now();
    let clock = mcu
        .call_msg::<_, crate::core::klippy::cmd::ClockState>(&GetClock, CALL_TIMEOUT)
        .await
        .map_err(|err| std::io::Error::other(format!("get_clock: {err}")))?
        .clock;
    // Time the round trip just made; when it cannot be measured, fall back to
    // `Duration::ZERO`, which `anchor_margin` turns back into the old 1 ms.
    let rtt = Instant::now()
        .checked_duration_since(sampled_at)
        .unwrap_or(Duration::ZERO);
    let margin = mcu
        .seconds_to_clock(anchor_margin(rtt).as_secs_f64())
        .map_err(|err| std::io::Error::other(format!("seconds_to_clock: {err}")))?
        as u32;
    mcu.send_msg(&ResetStepClock {
        oid,
        clock: clock.wrapping_add(margin),
    })
    .map_err(|err| std::io::Error::other(format!("reset_step_clock: {err}")))
}

/// A section's string parameter, trimmed.
fn section_string(section: &ConfigSection, key: &str) -> Option<String> {
    section
        .parameters
        .get(key)
        .and_then(|value| value.as_str_ref())
        .map(|value| value.trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::ConfigValue;

    fn section(id: &str, sub: Option<&str>, pairs: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, sub);
        for (key, value) in pairs {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn an_mcu_section_is_found_by_its_name() {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));

        assert!(find_mcu_section(&config, "mcu").is_some());
        assert_eq!(
            find_mcu_section(&config, "zboard").unwrap().sub.as_deref(),
            Some("zboard")
        );
        assert!(find_mcu_section(&config, "nope").is_none());
    }

    #[test]
    fn a_named_mcu_is_not_the_bare_one() {
        let mut config = Config::new();
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));
        assert!(find_mcu_section(&config, "mcu").is_none());
        assert!(find_mcu_section(&config, "zboard").is_some());
    }

    #[test]
    fn an_empty_mcu_name_means_the_bare_one() {
        assert_eq!(mcu_name(""), "mcu");
        assert_eq!(mcu_name("   "), "mcu");
        assert_eq!(mcu_name("zboard"), "zboard");
        assert_eq!(mcu_name(" zboard "), "zboard");

        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        assert!(find_mcu_section(&config, mcu_name("")).is_some());
    }

    #[test]
    fn a_stepper_on_another_mcu_is_skipped() {
        // `find_steppers` needs a connected MCU to resolve pins, so this only
        // checks that an empty config selects nothing at all.
        let config = Config::new();
        assert!(config.sections().next().is_none());
    }

    #[test]
    fn the_anchor_margin_covers_the_round_trip_plus_slack() {
        for rtt in [
            Duration::ZERO,
            Duration::from_millis(1),
            Duration::from_millis(4),
            Duration::from_millis(50),
        ] {
            let margin = anchor_margin(rtt);
            assert!(
                margin >= 2 * rtt + ANCHOR_SLACK,
                "rtt {rtt:?}: {margin:?} is under 2×rtt + slack"
            );
            assert!(
                margin >= Duration::from_millis(1),
                "rtt {rtt:?}: {margin:?} is under the 1 ms floor"
            );
        }
    }

    #[test]
    fn the_anchor_margin_keeps_the_old_fixed_lead_at_zero_rtt() {
        // Unmeasurable round trip → exactly the margin the tool used before.
        assert_eq!(anchor_margin(Duration::ZERO), Duration::from_millis(1));
        // Sanity on the formula's exact values on a real link.
        assert_eq!(
            anchor_margin(Duration::from_millis(4)),
            Duration::from_millis(9)
        );
        assert_eq!(
            anchor_margin(Duration::from_millis(50)),
            Duration::from_millis(101)
        );
    }

    #[test]
    fn a_stage_fills_its_duration_with_uniform_commands() {
        // 100 kHz on a 100 MHz clock is 1000 ticks per step. A 10 ms slice is
        // 1000 steps per command, and 50 commands make the 0.5 s stage.
        let stage = stage_for(100_000.0, 100_000_000.0, 0.5);
        assert_eq!(stage.interval, 1000);
        assert_eq!(stage.count, 1000);
        assert_eq!(stage.commands, 50);
        assert_eq!(stage.steps, 50_000);
        assert_eq!(stage.duration_us, 500_000);
    }

    #[test]
    fn a_slower_clock_produces_a_longer_interval_at_the_same_duration() {
        let stage = stage_for(1_000.0, 100_000_000.0, 0.5);
        assert_eq!(stage.interval, 100_000);
        assert_eq!(stage.count, 10);
        assert_eq!(stage.duration_us, 500_000);
    }

    #[test]
    fn the_interval_never_reaches_zero_and_commands_stay_bounded() {
        // A rate above the clock would divide to zero ticks; the firmware would
        // then schedule the next step in the past.
        let stage = stage_for(1e12, 100_000_000.0, 0.5);
        assert_eq!(stage.interval, 1);
        assert_eq!(stage.count, u16::MAX);
        assert_eq!(stage.commands, MOVE_SLOTS);
    }

    /// A pin registry with a chip for every named board, the way a run builds
    /// one: the same registry [`find_steppers`] and the stepper resource resolve
    /// through.
    fn fixture_pins(boards: &[&str]) -> Arc<PrinterPins> {
        let pins = Arc::new(PrinterPins::new());
        for name in boards {
            let chip = McuChip::new(
                (*name).to_string(),
                Arc::new(ConfigBuilder::new()),
                Arc::clone(&pins),
            );
            pins.register_chip(name, Arc::new(chip)).unwrap();
        }
        pins
    }

    /// The registry parses the description production parses — leading `!`,
    /// `mcu:`/`chip:` prefixes, and `[board_pins]` aliases — and the number comes
    /// from the firmware's own `pin` enumeration.
    #[tokio::test]
    async fn a_pin_resolves_through_the_firmware_enumeration() {
        use crate::core::klippy::interface::devices::frame_mock::FrameMock;
        use crate::core::klippy::interface::Interface;
        use crate::core::klippy::mcu::{Dictionary, Mcu};

        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({
                "enumerations": { "pin": { "PA0": 0, "PB1": 7 } }
            }))
            .unwrap(),
        )
        .unwrap();
        let pins = fixture_pins(&["mcu", "zboard"]);
        let number = |pins: &PrinterPins, description: &str| {
            let params = pins.parse_pin(description, true, false).unwrap();
            resolve_number(pins, &mcu, &params.chip_name, &params)
        };

        assert_eq!(number(&pins, "PA0"), Ok(0));
        // The chip prefix is the pinned form, and a leading `!` is invert, not
        // part of the name — `!PA0` names PA0.
        assert_eq!(number(&pins, "mcu:PB1"), Ok(7));
        assert_eq!(number(&pins, "!PB1"), Ok(7));
        // A name the firmware does not know.
        assert!(number(&pins, "PC9").is_err());

        // An alias resolves to the pin it names. One pin used under two names
        // in the same run is upstream's `IsAlias` error, so this is its own
        // registry, where the alias is the only name the pin ever had.
        let pins = fixture_pins(&["mcu"]);
        pins.alias_pin("mcu", "X_STEP", "PA0").unwrap();
        assert_eq!(number(&pins, "X_STEP"), Ok(0));
        // `!` comes before the chip prefix, not after it (`!mcu:X_STEP`).
        assert_eq!(number(&pins, "!mcu:X_STEP"), Ok(0));
    }

    // -----------------------------------------------------------------------
    // Multi-MCU runs
    // -----------------------------------------------------------------------

    /// A config with a bare `[mcu]`, a named one, and a section that is not an
    /// MCU at all (so `--all-mcus` has something to skip).
    fn two_board_config() -> Config {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));
        config.add_section(section("printer", None, &[("kinematics", "cartesian")]));
        config
    }

    /// Which boards a command line selects, how a name resolves, and how the
    /// report lines and summaries of concurrent boards stay attributable.
    #[test]
    fn mcus_are_selected_by_name_or_all_and_summarised_per_board() {
        let config = two_board_config();

        // No name given → the single default `mcu`, exactly as before `--all-mcus`.
        assert_eq!(select_mcus(&config, &[], false).unwrap(), ["mcu"]);
        // Names on the command line, in order; an empty one is still the bare
        // `[mcu]` section.
        assert_eq!(
            select_mcus(&config, &["zboard".to_string()], false).unwrap(),
            ["zboard"]
        );
        assert_eq!(
            select_mcus(&config, &["".to_string(), "zboard".to_string()], false).unwrap(),
            ["mcu", "zboard"]
        );
        // A board named twice is driven once.
        assert_eq!(
            select_mcus(&config, &["mcu".to_string(), "mcu".to_string()], false).unwrap(),
            ["mcu"]
        );
        // `--all-mcus` enumerates every `[mcu]` section in config order (the
        // bare one first) and skips sections that are not MCUs …
        assert_eq!(select_mcus(&config, &[], true).unwrap(), ["mcu", "zboard"]);
        // … and it does not take names of its own.
        assert!(select_mcus(&config, &["zboard".to_string()], true).is_err());

        // Each name resolves to its own section; a missing one says so.
        assert_eq!(resolve_mcu(&config, "mcu").unwrap().name, "mcu");
        assert_eq!(resolve_mcu(&config, "zboard").unwrap().name, "zboard");
        assert_eq!(
            resolve_mcu(&config, "nope").unwrap_err(),
            "no [mcu nope] section"
        );

        // Report lines carry their board's prefix, the closing summaries too.
        assert_eq!(tagged("zboard", "queued"), "[zboard] queued");
        assert_eq!(
            summary_line("zboard", "survived", Some(12_500.0), "steps/s"),
            "[zboard] last rate it survived: 12500 steps/s"
        );
        assert_eq!(
            summary_line("mcu", "carried", None, "req/s"),
            "[mcu] last rate it carried: none"
        );
    }

    /// The `stress` subcommand's arguments, so a test can parse command lines.
    #[derive(clap::Parser)]
    struct Cli {
        #[command(flatten)]
        stress: StressArgs,
    }

    /// The old single-board command line still means the default `mcu`, and the
    /// new spellings add boards without changing what the old ones select.
    #[test]
    fn the_old_single_board_command_line_still_selects_the_default_mcu() {
        use clap::Parser;

        let config = two_board_config();

        // The documented invocation: a config file and no MCU name at all.
        let cli = Cli::try_parse_from(["stress", "printer.cfg"]).unwrap();
        assert!(cli.stress.mcu.is_empty(), "no name was given");
        assert!(!cli.stress.all_mcus);
        assert_eq!(cli.stress.task, Task::Step);
        assert_eq!(
            select_mcus(&config, &cli.stress.mcu, cli.stress.all_mcus).unwrap(),
            ["mcu"]
        );

        // One name keeps meaning exactly that board.
        let cli = Cli::try_parse_from(["stress", "printer.cfg", "zboard"]).unwrap();
        assert_eq!(cli.stress.mcu, ["zboard"]);
        assert_eq!(
            select_mcus(&config, &cli.stress.mcu, cli.stress.all_mcus).unwrap(),
            ["zboard"]
        );

        // Several names at once, with a flag after them.
        let cli = Cli::try_parse_from(["stress", "printer.cfg", "mcu", "zboard", "--task", "comm"])
            .unwrap();
        assert_eq!(cli.stress.mcu, ["mcu", "zboard"]);
        assert_eq!(cli.stress.task, Task::Comm);

        // `--all-mcus` alone enumerates the config's sections.
        let cli = Cli::try_parse_from(["stress", "printer.cfg", "--all-mcus"]).unwrap();
        assert!(cli.stress.all_mcus);
        assert!(cli.stress.mcu.is_empty());
        assert_eq!(
            select_mcus(&config, &cli.stress.mcu, cli.stress.all_mcus).unwrap(),
            ["mcu", "zboard"]
        );
    }

    /// Two fake boards on one runtime: each runs the real `identify` handshake
    /// against its own dictionary-driven device, then is driven with round-trips
    /// and a batch of fire-and-forget commands at the same time as the other.
    ///
    /// What this pins down is that the two sessions never share anything: each
    /// board ends up with *its own* dictionary (identify never crossed), and
    /// each batch of `get_clock` requests is answered exactly once, on the board
    /// that sent it. A full ramp against two fake boards would just measure the
    /// fake — that is what the real boards are for.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn two_fake_boards_identify_and_drive_concurrently() {
        let dict_a = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let dict_b = klipperx_test_support::test_dicts_dir().join("linuxprocess.dict");
        let text = format!(
            "[mcu]\ntest: dict={}\n\n[mcu zboard]\ntest: dict={}\n",
            dict_a.display(),
            dict_b.display()
        );
        let (config, _) = Config::from_text(&text).expect("two fake MCUs parse");

        // What `--all-mcus` selects, then one connection (identify) per board,
        // running concurrently rather than one after the other.
        assert_eq!(select_mcus(&config, &[], true).unwrap(), ["mcu", "zboard"]);
        let connects = ["mcu", "zboard"]
            .iter()
            .map(|name| {
                let mcu_config = resolve_mcu(&config, name).unwrap();
                async move { connect(&mcu_config).await }
            })
            .collect::<Vec<_>>();
        let mut connected = join_all(connects).await.into_iter();
        let mcu_a = connected
            .next()
            .unwrap()
            .expect("the bare [mcu] identifies");
        let mcu_b = connected.next().unwrap().expect("[mcu zboard] identifies");

        // Each board carries its own dictionary: the two files differ in their
        // command counts, so a crossed identify would be visible here.
        let command_count = |path: &std::path::Path| {
            let json: serde_json::Value = serde_json::from_slice(
                &std::fs::read(path).unwrap_or_else(|e| panic!("{path:?}: {e}")),
            )
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            json.get("commands")
                .and_then(|commands| commands.as_object())
                .map(|commands| commands.len())
                .unwrap_or(0)
        };
        let commands_a = mcu_a
            .dictionary()
            .expect("board A identified")
            .commands()
            .len();
        let commands_b = mcu_b
            .dictionary()
            .expect("board B identified")
            .commands()
            .len();
        assert_eq!(
            commands_a,
            command_count(&dict_a),
            "board A got its own dictionary"
        );
        assert_eq!(
            commands_b,
            command_count(&dict_b),
            "board B got its own dictionary"
        );
        assert_ne!(
            commands_a, commands_b,
            "the two dictionaries must differ for this to prove which board is which"
        );
        assert_eq!(mcu_a.name(), "mcu");
        assert_eq!(mcu_b.name(), "zboard");

        /// Five round-trips, then a fire-and-forget batch whose answers are
        /// counted by a bound callback — the shape `comm_stress` runs, cut
        /// short so a fake board can carry it.
        async fn drive(mcu: Arc<Mcu>) -> Result<u64, Box<dyn std::error::Error>> {
            use crate::core::klippy::cmd::ClockState;

            const ROUNDS: usize = 5;
            const BATCH: u64 = 25;

            let mut clocks = Vec::new();
            for _ in 0..ROUNDS {
                clocks.push(
                    mcu.call_msg::<GetClock, ClockState>(&GetClock, CALL_TIMEOUT)
                        .await?
                        .clock,
                );
            }
            assert!(
                clocks.windows(2).all(|pair| pair[0] <= pair[1]),
                "board '{}' answers from its own monotonic clock: {clocks:?}",
                mcu.name()
            );

            let answered = Arc::new(AtomicU64::new(0));
            {
                let answered = Arc::clone(&answered);
                mcu.bind_callback("clock", move |_| {
                    answered.fetch_add(1, Ordering::Relaxed);
                })?;
            }
            for _ in 0..BATCH {
                mcu.send_msg(&GetClock)?;
            }
            mcu.flush(CALL_TIMEOUT).await?;
            let deadline = Instant::now() + Duration::from_secs(5);
            while answered.load(Ordering::Relaxed) < BATCH && Instant::now() < deadline {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Ok(answered.load(Ordering::Relaxed))
        }

        let drives = join_all(vec![drive(mcu_a.clone()), drive(mcu_b.clone())]).await;
        for (name, outcome) in ["mcu", "zboard"].iter().zip(&drives) {
            let answered = outcome
                .as_ref()
                .unwrap_or_else(|err| panic!("{name}'s drive failed: {err}"));
            assert_eq!(
                *answered, 25,
                "{name} answered its own batch, not the other's"
            );
        }
    }

    /// Two sessions over their own scripted ports, driven at the same time:
    /// every frame each sends lands in that session's own recorder, in order,
    /// and the sequence numbers stay in the session that produced them.
    ///
    /// The two boards speak different commands (`get_clock` vs `get_uptime`),
    /// so a frame on the wrong port would show up as a foreign payload in one
    /// recorder — and would not match the scripted input, so the session could
    /// not complete. A full ramp needs a real board; this covers the part a fake
    /// can prove: the wiring keeps sessions apart while both are busy.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn concurrent_sessions_send_only_their_own_frames_to_their_own_recorder() {
        use crate::core::klippy::cmd::ClockState;
        use crate::core::klippy::frame::Frame;
        use crate::core::klippy::interface::devices::frame_mock::{
            FrameMock, FrameRecorder, MappingEntry,
        };
        use crate::core::klippy::interface::Interface;
        use crate::core::klippy::mcu::{Dictionary, Mcu};

        use crate::core::klippy::msg::proto::Payload;
        const ROUNDS: u8 = 8;

        /// A device scripted for `ROUNDS` blocks carrying `command`: each is
        /// acknowledged the way the firmware does it (an empty frame numbered
        /// `seq + 1`), and the last one also carries `response`, so the trailing
        /// `call` has an answer.
        fn scripted(command: u8, response: &[u8]) -> (FrameMock, FrameRecorder) {
            let mut mapping = Vec::new();
            for seq in 0..ROUNDS {
                let mut outputs = vec![Frame::new(seq + 1, Vec::new())];
                if seq + 1 == ROUNDS {
                    outputs.insert(0, Frame::new(seq + 1, response.to_vec()));
                }
                mapping.push(MappingEntry {
                    input: Frame::new(seq, vec![command]),
                    outputs,
                });
            }
            let device = FrameMock::new(mapping);
            let recorder = device.recorder();
            (device, recorder)
        }

        // `clock clock=%u` is response 18; `uptime high=%u clock=%u` is 17.
        // The id and the `%u` values are VLQ-encoded, which is what `Payload`
        // does — raw little-endian bytes would not decode.
        let mut clock_answer = Payload::new();
        clock_answer.push_i16(18).unwrap();
        clock_answer.push_u32(123_456).unwrap();
        let clock_answer = clock_answer.into_raw();
        let mut uptime_answer = Payload::new();
        uptime_answer.push_i16(17).unwrap();
        uptime_answer.push_u32(7).unwrap();
        uptime_answer.push_u32(654_321).unwrap();
        let uptime_answer = uptime_answer.into_raw();

        let (device_a, recorder_a) = scripted(5, &clock_answer);
        let (device_b, recorder_b) = scripted(6, &uptime_answer);
        let mcu_a = Arc::new(Mcu::for_test("board_a", Interface::new(device_a)));
        let mcu_b = Arc::new(Mcu::for_test("board_b", Interface::new(device_b)));
        mcu_a
            .install_dictionary(
                Dictionary::from_json(serde_json::json!({
                    "commands": {"get_clock": 5},
                    "responses": {"clock clock=%u": 18}
                }))
                .unwrap(),
            )
            .unwrap();
        mcu_b
            .install_dictionary(
                Dictionary::from_json(serde_json::json!({
                    "commands": {"get_uptime": 6},
                    "responses": {"uptime high=%u clock=%u": 17}
                }))
                .unwrap(),
            )
            .unwrap();

        /// Send `ROUNDS - 1` fire-and-forget commands (one flush each, so each
        /// becomes its own block), then close with one round-trip.
        async fn drive(mcu: Arc<Mcu>, clock: bool) -> Result<(), Box<dyn std::error::Error>> {
            for _ in 0..ROUNDS - 1 {
                if clock {
                    mcu.send_msg(&GetClock)?;
                } else {
                    mcu.send_msg(&GetUptime)?;
                }
                mcu.flush(CALL_TIMEOUT).await?;
            }
            if clock {
                let state = mcu
                    .call_msg::<GetClock, ClockState>(&GetClock, CALL_TIMEOUT)
                    .await?;
                assert_eq!(state.clock, 123_456, "answered by its own scripted port");
            } else {
                let uptime = mcu
                    .call_msg::<GetUptime, Uptime>(&GetUptime, CALL_TIMEOUT)
                    .await?;
                assert_eq!(
                    (uptime.high, uptime.clock),
                    (7, 654_321),
                    "answered by its own scripted port"
                );
            }
            Ok(())
        }

        let sent = |recorder: &FrameRecorder| -> Vec<(u8, Vec<u8>)> {
            recorder
                .frames()
                .iter()
                .map(|frame| (frame.seq(), frame.payload().to_vec()))
                .collect()
        };
        let expected = |command: u8| -> Vec<(u8, Vec<u8>)> {
            (0..ROUNDS).map(|seq| (seq, vec![command])).collect()
        };

        let (a, b) = tokio::join!(drive(mcu_a, true), drive(mcu_b, false));
        a.expect("session A completes without an error");
        b.expect("session B completes without an error");

        // Every frame stayed where it belongs: A's recorder holds only
        // `get_clock` blocks and B's only `get_uptime` blocks, each numbered
        // 0..ROUNDS-1 in order — no foreign payload, no duplicated or skipped
        // sequence number.
        assert_eq!(
            sent(&recorder_a),
            expected(5),
            "board A's own frames, in order"
        );
        assert_eq!(
            sent(&recorder_b),
            expected(6),
            "board B's own frames, in order"
        );
    }

    // -----------------------------------------------------------------------
    // Reset → reconnect closes the old session (B2)
    // -----------------------------------------------------------------------

    /// The `ResetRequired` reconnect the way hardware reached it (vA): a board
    /// still carrying a **different** configuration can only be taken over by
    /// rebooting it (`reset`), and the old session has to be closed before the
    /// port is reopened. The run's hand-off ([`drive_boards`]) is what makes
    /// that close real — with a caller-side `Arc<Mcu>` clone the `drop` in
    /// [`configure_steppers`] would release nothing and the old receive task
    /// would stay a second reader on the port through `reconnect()`.
    ///
    /// The contention itself (two readers on **one** transport) is not
    /// expressible against a fake: every session gets its own device from
    /// `McuConfig::open`, so no frame can be stolen. What this pins down is the
    /// invariant behind it — the reconnect is the old session's last owner
    /// (asserted where the drop happens), that session is really dropped, and
    /// the new session identifies, is configured and drives its ramp to the end.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_reset_reconnect_drops_the_old_session_before_reopening() {
        use clap::Parser;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let text = format!(
            "[mcu]\ntest: dict={}\n\n[stepper_x]\nstep_pin: PA0\ndir_pin: PB0\n",
            dict.display()
        );
        let (config, _) = Config::from_text(&text).expect("a fake board with a stepper parses");
        let mcu_config = resolve_mcu(&config, "mcu").expect("the [mcu] section resolves");

        // The reset path's preconditions: this firmware's only reset is the
        // `reset` command (`mcu/config.rs` answers `ResetRequired` for a board
        // that carries a different CRC), and the board already carries one.
        let mcu = connect(&mcu_config)
            .await
            .expect("the fake board identifies");
        assert!(
            mcu.has_message(Reset::NAME),
            "the fixture's firmware only reboots with 'reset'"
        );
        let stale = ConfigBuilder::new();
        stale
            .configure(&mcu)
            .await
            .expect("the board takes an initial configuration");
        // The CRC it now carries: an empty configuration encodes the same bytes
        // every time, so a second empty builder reads back that same value.
        let carried_crc = ConfigBuilder::new()
            .build(&mcu)
            .expect("an empty configuration builds")
            .crc;
        // The configuration the run will send — the same registry and the same
        // real stepper resource — so its CRC is the one the handshake compares
        // against. This probe uses a throwaway registry: the one the run builds
        // inside `stress` must still be unfinalized when it gets there.
        let probe = build_pin_registry(&config).expect("the pin registry builds");
        let probe_builder = probe.chip_as::<McuChip>("mcu").unwrap().config();
        probe
            .setup_stepper("PA0", "PB0", DEFAULT_STEP_PULSE_DURATION)
            .expect("the stress stepper fits");
        for _ in 0..MOVE_SLOTS {
            probe_builder.request_move_queue_slot().unwrap();
        }
        let stress_crc = probe_builder
            .build(&mcu)
            .expect("the stress configuration builds")
            .crc;
        assert_ne!(
            stress_crc, carried_crc,
            "a matching CRC would reuse the configuration and never reset"
        );

        // The hand-off a run makes: the session moves into the board's future
        // and only a `Weak` stays outside to watch it.
        let old_session = Arc::downgrade(&mcu);
        let args = Cli::try_parse_from([
            "stress",
            "printer.cfg",
            "--stage-seconds",
            "0.01",
            "--rate-step",
            "100",
        ])
        .expect("a short, coarse ramp parses")
        .stress;
        let pins = build_pin_registry(&config).expect("the pin registry builds");
        let (failures, ran) = drive_boards(vec![(mcu_config, mcu)], &config, &args, &pins).await;
        assert!(
            failures.is_empty(),
            "the board's ramp does not fail: {failures:?}"
        );
        assert_eq!(
            ran, 1,
            "the reconnected session identified, configured and drove its ramp"
        );

        // The old session is gone: `configure_steppers` dropped it at the
        // reconnect — the preconditions above make that path unavoidable, and
        // its `Arc::strong_count` assertion runs right where the drop happens.
        assert!(
            old_session.upgrade().is_none(),
            "the old session must be dropped before the port is reopened"
        );
    }

    // -----------------------------------------------------------------------
    // End to end against a `test: dict=` fake board
    // -----------------------------------------------------------------------

    /// A whole `--task step` run against a `test: dict=` fake: identify →
    /// configure (through the real stepper resource) → the ramp's `queue_step`
    /// stages. The pins are reached through `[board_pins]` aliases with a leading
    /// `!`, so this is the path a real invocation takes, end to end.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_step_ramp_runs_end_to_end_against_a_fake_board() {
        use clap::Parser;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let path =
            std::env::temp_dir().join(format!("klipperx-stress-step-{}.cfg", std::process::id()));
        std::fs::write(
            &path,
            format!(
                "[mcu]\ntest: dict={}\n\n\
                 [board_pins]\nmcu: mcu\naliases: X_STEP=PA0, X_DIR=PB0, X_EN=PC0\n\n\
                 [stepper_x]\nstep_pin: X_STEP\ndir_pin: X_DIR\nenable_pin: !X_EN\n\
                 microsteps: 16\nrotation_distance: 40\n",
                dict.display()
            ),
        )
        .expect("the config file is written");
        let args = Cli::try_parse_from([
            "stress",
            path.to_str().expect("the temp path is utf-8"),
            "--stage-seconds",
            "0.01",
            "--rate-step",
            "100",
        ])
        .expect("a short, coarse ramp parses")
        .stress;
        let outcome = stress(args).await;
        let _ = std::fs::remove_file(&path);

        // Whatever the ramp finds — the fake's step model can shut down, or the
        // ramp can reach its top — a shutdown is the measurement, not an error.
        outcome.expect("the step ramp ran to the end of the command");
    }

    /// A whole `--task comm` ramp against a `test: dict=` fake: identify, then
    /// `get_clock` round-trips at a rising request rate. The comm task never
    /// configures the firmware, so this only needs the transport.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_comm_ramp_runs_end_to_end_against_a_fake_board() {
        use clap::Parser;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let text = format!("[mcu]\ntest: dict={}\n", dict.display());
        let (config, _) = Config::from_text(&text).expect("a fake board parses");
        let mcu_config = resolve_mcu(&config, "mcu").expect("the [mcu] section resolves");
        let mcu = connect(&mcu_config)
            .await
            .expect("the fake board identifies");
        let args = Cli::try_parse_from([
            "stress",
            "printer.cfg",
            "--task",
            "comm",
            "--stage-seconds",
            "0.5",
            "--rate-step",
            "3000",
        ])
        .expect("a short comm ramp parses")
        .stress;
        let pins = build_pin_registry(&config).expect("the pin registry builds");

        let (failures, ran) = drive_boards(vec![(mcu_config, mcu)], &config, &args, &pins).await;
        assert!(failures.is_empty(), "{failures:?}");
        assert_eq!(ran, 1, "the comm ramp identified and ran its stages");
    }

    /// The run hands every session to its board future and keeps no reference
    /// of its own — the caller-side half of the same invariant. `run_board`
    /// checks `Arc::strong_count` the moment its future runs, so a clone
    /// restored at the call site (the shape that let vA's old receive task
    /// live through every reopen) fails here rather than on hardware.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_run_gives_each_session_to_its_board_future_outright() {
        use clap::Parser;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let path =
            std::env::temp_dir().join(format!("klipperx-stress-m3-{}.cfg", std::process::id()));
        std::fs::write(&path, format!("[mcu]\ntest: dict={}\n", dict.display()))
            .expect("the config file is written");
        let args = Cli::try_parse_from([
            "stress",
            path.to_str().expect("the temp path is utf-8"),
            "--all-mcus",
        ])
        .expect("the command line parses")
        .stress;
        let outcome = stress(args).await;
        let _ = std::fs::remove_file(&path);

        // No stepper section anywhere: the enumerated board is skipped and the
        // step task reports that it had nothing to drive — the run itself went
        // through `run_board`'s sole-owner assertion on the way.
        let err = outcome.expect_err("a run with no stepper section fails");
        assert!(err.to_string().contains("nothing to drive"), "{err}");
    }

    // -----------------------------------------------------------------------
    // Every stepper of a board (`find_steppers` / the step task's plan)
    // -----------------------------------------------------------------------

    /// A connected-looking MCU whose dictionary knows every pin name the
    /// fixtures below use, so selection can resolve what it collects.
    fn fixture_mcu() -> Mcu {
        use crate::core::klippy::interface::devices::frame_mock::FrameMock;
        use crate::core::klippy::interface::Interface;
        use crate::core::klippy::mcu::Dictionary;

        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({
                "enumerations": { "pin": {
                    "PA0": 0, "PA1": 1, "PA2": 2, "PA3": 3, "PA4": 4,
                    "PB0": 10, "PB1": 11, "PB2": 12,
                    "PC0": 20, "PC1": 21
                } }
            }))
            .unwrap(),
        )
        .unwrap();
        mcu
    }

    /// The section names a selection carries, for compact assertions.
    fn sections(steppers: &[StepperSource]) -> Vec<&str> {
        steppers
            .iter()
            .map(|section| section.name.as_str())
            .collect()
    }

    /// Ownership is `step_pin`'s chip: a board takes **every** section whose
    /// step pin is its own, and neither an `enable_pin` nor an `endstop_pin`
    /// on another board hands a section over to it.
    #[tokio::test]
    async fn a_stepper_belongs_to_the_board_whose_step_pin_it_uses() {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));
        config.add_section(section(
            "stepper_x",
            None,
            &[
                ("step_pin", "mcu:PA0"),
                ("dir_pin", "mcu:PA1"),
                ("enable_pin", "mcu:PA2"),
            ],
        ));
        // Its endstop sits on the other board — that does not make it theirs.
        config.add_section(section(
            "stepper_z",
            None,
            &[
                ("step_pin", "zboard:PB0"),
                ("dir_pin", "zboard:PB1"),
                ("endstop_pin", "mcu:PC0"),
            ],
        ));
        // Unprefixed pins name the bare `[mcu]`, and `manual_stepper` counts.
        config.add_section(section(
            "manual_stepper",
            Some("docking"),
            &[("step_pin", "PA3"), ("dir_pin", "PA4")],
        ));
        let mcu = fixture_mcu();

        // The primary board owns both of *its* sections — not just the first.
        let pins = fixture_pins(&["mcu", "zboard"]);
        let (mine, rejected) = find_steppers(&config, "mcu", &mcu, &pins);
        assert!(rejected.is_empty(), "{rejected:?}");
        assert_eq!(sections(&mine), ["stepper_x", "manual_stepper docking"]);
        assert_eq!(mine[0].numbers, (0, 1), "PA0/PA1 resolved");
        assert_eq!(mine[1].numbers, (3, 4), "PA3/PA4 resolved");

        // … and only its own: `stepper_z`'s endstop on `mcu` pulls nothing
        // towards the primary board, nor `stepper_x`'s enable towards zboard.
        let (theirs, rejected) = find_steppers(&config, "zboard", &mcu, &pins);
        assert!(rejected.is_empty(), "{rejected:?}");
        assert_eq!(sections(&theirs), ["stepper_z"]);
        assert_eq!(theirs[0].numbers, (10, 11), "PB0/PB1 resolved");
    }

    /// A `dir_pin` on another MCU rejects that one section — with upstream's
    /// wording — without sinking the board's other steppers, and without
    /// handing the section to the board the dir pin names.
    #[tokio::test]
    async fn a_dir_pin_on_another_board_rejects_only_that_section() {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));
        config.add_section(section(
            "stepper_bad",
            None,
            &[("step_pin", "mcu:PA0"), ("dir_pin", "zboard:PB1")],
        ));
        config.add_section(section(
            "stepper_ok",
            None,
            &[("step_pin", "mcu:PA2"), ("dir_pin", "mcu:PA3")],
        ));
        let mcu = fixture_mcu();
        let pins = fixture_pins(&["mcu", "zboard"]);

        // One broken section is rejected; the board's other stepper still runs.
        let (mine, rejected) = find_steppers(&config, "mcu", &mcu, &pins);
        assert_eq!(sections(&mine), ["stepper_ok"]);
        assert_eq!(rejected.len(), 1, "{rejected:?}");
        assert!(rejected[0].starts_with("stepper_bad: "), "{rejected:?}");
        assert!(
            rejected[0].contains("Stepper dir pin must be on same mcu as step pin"),
            "{rejected:?}"
        );

        // Ownership stays `step_pin`'s chip: the dir pin's board does not claim it.
        let (theirs, _) = find_steppers(&config, "zboard", &mcu, &pins);
        assert!(theirs.is_empty(), "a section is never the other board's");

        // A board whose *only* section is broken: named → hard error carrying
        // the wording; enumerated by `--all-mcus` → skipped instead.
        let mut broken_only = Config::new();
        broken_only.add_section(section(
            "stepper_bad",
            None,
            &[("step_pin", "mcu:PA0"), ("dir_pin", "zboard:PB1")],
        ));
        let (none, rejected) = find_steppers(&broken_only, "mcu", &mcu, &pins);
        assert!(none.is_empty());
        assert_eq!(rejected.len(), 1, "{rejected:?}");
        match plan_stepper_task("mcu", none, &rejected, true) {
            StepPlan::Fail(message) => assert!(
                message.contains("Stepper dir pin must be on same mcu as step pin"),
                "{message}"
            ),
            plan => panic!("a named board must fail on its broken section: {plan:?}"),
        }
        match plan_stepper_task("mcu", vec![], &rejected, false) {
            StepPlan::Skip(line) => assert_eq!(
                line,
                "no usable stepper section on this MCU; skipping step task"
            ),
            plan => panic!("an enumerated board must be skipped: {plan:?}"),
        }
    }

    /// The core selection chain: with a stepper on one board only,
    /// `--all-mcus --task step` drives that board and skips the other — while
    /// naming the empty board yourself is still the hard error it always was.
    #[tokio::test]
    async fn all_mcus_drives_the_board_with_a_stepper_and_skips_the_one_without() {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "mcu",
            Some("zboard"),
            &[("serial", "/dev/ttyACM1")],
        ));
        config.add_section(section(
            "stepper_x",
            None,
            &[("step_pin", "PA0"), ("dir_pin", "PA1")],
        ));
        let mcu = fixture_mcu();
        let pins = fixture_pins(&["mcu", "zboard"]);

        // `--all-mcus` enumerates boards it did not name (`named` is false).
        let (primary, primary_rejected) = find_steppers(&config, "mcu", &mcu, &pins);
        match plan_stepper_task("mcu", primary, &primary_rejected, false) {
            StepPlan::Drive(steppers) => assert_eq!(sections(&steppers), ["stepper_x"]),
            plan => panic!("the board with a stepper must be driven: {plan:?}"),
        }
        let (other, other_rejected) = find_steppers(&config, "zboard", &mcu, &pins);
        match plan_stepper_task("zboard", other, &other_rejected, false) {
            StepPlan::Skip(line) => {
                assert_eq!(line, "no stepper section on this MCU; skipping step task")
            }
            plan => panic!("the board without one must be skipped, not failed: {plan:?}"),
        }

        // Naming that board yourself is a hard error, with the message a board
        // without a stepper has always got — the docs quote it.
        match plan_stepper_task("zboard", vec![], &[], true) {
            StepPlan::Fail(message) => assert!(
                message.contains(
                    "no [stepper_*] or [manual_stepper] section on MCU 'zboard' to take a step/dir pin pair from"
                ),
                "{message}"
            ),
            plan => panic!("a named board without a stepper must fail: {plan:?}"),
        }
    }

    /// Both of a board's steppers are driven by the same stage: one oid each,
    /// the same schedule, `stage.commands` apiece — this is the list
    /// [`queue_stage`] puts on the wire.
    #[test]
    fn one_stage_drives_every_stepper_of_the_board_with_the_same_schedule() {
        let stage = Stage {
            interval: 1000,
            count: 100,
            commands: 3,
            steps: 300,
            duration_us: 30_000,
        };
        let commands = stage_commands(&[3, 7], &stage);
        assert_eq!(commands.len(), 2 * stage.commands as usize);
        for oid in [3u8, 7] {
            let own = commands.iter().filter(|command| command.oid == oid).count();
            assert_eq!(own, stage.commands as usize, "stepper {oid}");
        }
        for command in &commands {
            assert_eq!(command.interval, stage.interval);
            assert_eq!(command.count, stage.count);
            assert_eq!(command.add, 0);
        }
        // Round by round: stepper 3's batch, then stepper 7's — not one
        // stepper's whole stage ahead of the other's.
        assert_eq!(
            commands
                .iter()
                .map(|command| command.oid)
                .collect::<Vec<_>>(),
            vec![3, 3, 3, 7, 7, 7]
        );

        // The single-stepper shape is unchanged: one oid, one copy.
        let single = stage_commands(&[0], &stage);
        assert_eq!(single.len(), stage.commands as usize);
        assert!(single.iter().all(|command| command.oid == 0));
    }

    /// Two steppers in one configuration round: each gets its own oid and its
    /// own move-queue reserve, and both `config_stepper` commands go into the
    /// **same** `finalize_config` (a second round would `config_reset` the
    /// board and drop the first stepper).
    #[tokio::test]
    async fn every_stepper_gets_its_own_oid_in_one_config_round() {
        use crate::core::klippy::interface::devices::frame_mock::FrameMock;
        use crate::core::klippy::interface::Interface;
        use crate::core::klippy::mcu::Dictionary;

        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({
                "commands": {
                    "allocate_oids count=%c": 40,
                    "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u": 41,
                    "reset_step_clock oid=%c clock=%u": 43,
                    "finalize_config crc=%u": 42
                },
                "enumerations": { "pin": { "PA0": 0, "PA1": 1, "PB0": 10, "PB1": 11 } },
                "config": { "CLOCK_FREQ": 20000000 }
            }))
            .unwrap(),
        )
        .unwrap();

        let pins = fixture_pins(&["mcu"]);
        let builder = pins.chip_as::<McuChip>("mcu").unwrap().config();
        let first = pins
            .setup_stepper("PA0", "PA1", DEFAULT_STEP_PULSE_DURATION)
            .expect("the first stepper fits");
        let second = pins
            .setup_stepper("PB0", "PB1", DEFAULT_STEP_PULSE_DURATION)
            .expect("the second fits too");
        for _ in 0..2 {
            for _ in 0..MOVE_SLOTS {
                builder.request_move_queue_slot().unwrap();
            }
        }
        let built = builder
            .build(&mcu)
            .expect("both steppers are one configuration");
        assert_ne!(
            first.oid().unwrap(),
            second.oid().unwrap(),
            "each stepper gets its own oid"
        );
        assert_eq!(
            built.move_slots,
            2 * MOVE_SLOTS as u16,
            "each stepper reserves its own stage's worth of the move queue"
        );
        assert_eq!(
            built.config.len(),
            4,
            "allocate_oids + 2 config_stepper + finalize_config"
        );
    }

    /// The firmware `config_stepper` wears the section's own step options: the
    /// step pin's `!` becomes upstream's `invert_step`, and
    /// `step_pulse_duration` becomes `step_pulse_ticks` — the two values the
    /// stress stepper used to hardcode as `0`.
    #[tokio::test]
    async fn the_stress_stepper_carries_the_sections_own_step_options() {
        use crate::core::klippy::frame::Frame;
        use crate::core::klippy::interface::devices::frame_mock::FrameMock;
        use crate::core::klippy::interface::Interface;
        use crate::core::klippy::mcu::Dictionary;
        use crate::core::klippy::msg::parser::Parser;
        use crate::core::klippy::msg::proto::ArgValue;

        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({
                "commands": {
                    "allocate_oids count=%c": 40,
                    "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u": 41,
                    "reset_step_clock oid=%c clock=%u": 43,
                    "finalize_config crc=%u": 42
                },
                "enumerations": { "pin": { "PA0": 0, "PB0": 10 } },
                "config": { "CLOCK_FREQ": 20000000 }
            }))
            .unwrap(),
        )
        .unwrap();

        let pins = fixture_pins(&["mcu"]);
        let builder = pins.chip_as::<McuChip>("mcu").unwrap().config();
        // `!PA0` is invert_step 1; a 4 µs pulse on a 20 MHz clock is 80 ticks.
        pins.setup_stepper("!PA0", "PB0", 0.000_004)
            .expect("the resource builds");
        for _ in 0..MOVE_SLOTS {
            builder.request_move_queue_slot().unwrap();
        }
        let built = builder.build(&mcu).expect("the configuration builds");

        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let payload = &built.config[1];
        let frame = Frame::new(0, payload.payload().to_vec());
        let decoded = parser.decode(frame.into()).unwrap();
        assert_eq!(decoded[0].0.name, "config_stepper");
        assert_eq!(
            decoded[0].1[3],
            ArgValue::UInt8(1),
            "invert_step follows the step pin's `!`"
        );
        assert_eq!(
            decoded[0].1[4],
            ArgValue::UInt32(80),
            "step_pulse_ticks follows step_pulse_duration"
        );
    }

    /// The selection reads `[board_pins]` aliases and a leading `!` through the
    /// shared registry, and takes `microsteps`/`rotation_distance` for the
    /// motion smoke's step distance.
    #[tokio::test]
    async fn board_pins_aliases_and_geometry_are_read_from_the_section() {
        let mut config = Config::new();
        config.add_section(section("mcu", None, &[("serial", "/dev/ttyACM0")]));
        config.add_section(section(
            "stepper_x",
            None,
            &[
                ("step_pin", "!X_STEP"),
                ("dir_pin", "X_DIR"),
                ("microsteps", "16"),
                ("rotation_distance", "40"),
            ],
        ));
        let mcu = fixture_mcu();
        let pins = fixture_pins(&["mcu"]);
        pins.alias_pin("mcu", "X_STEP", "PA0").unwrap();
        pins.alias_pin("mcu", "X_DIR", "PA1").unwrap();

        let (steppers, rejected) = find_steppers(&config, "mcu", &mcu, &pins);
        assert!(rejected.is_empty(), "{rejected:?}");
        assert_eq!(sections(&steppers), ["stepper_x"]);
        assert_eq!(
            steppers[0].numbers,
            (0, 1),
            "X_STEP/X_DIR resolved through the aliases"
        );
        // 40 mm per rotation over 200 full steps × 16 microsteps.
        assert_eq!(steppers[0].options.step_dist, Some(0.0125));

        // A section with no geometry at all still selects — the step task only
        // borrows the pins — and reports no step distance.
        let mut bare = Config::new();
        bare.add_section(section(
            "stepper_x",
            None,
            &[("step_pin", "PA0"), ("dir_pin", "PA1")],
        ));
        let (steppers, rejected) = find_steppers(&bare, "mcu", &mcu, &fixture_pins(&["mcu"]));
        assert!(rejected.is_empty(), "{rejected:?}");
        assert_eq!(steppers[0].options.step_dist, None);
    }

    /// The exit-code rules around skipping: a skipped board is not a failure,
    /// a run that skipped *every* board tested nothing, and a board that
    /// actually failed still fails the command over the selected boards.
    #[test]
    fn a_skipped_board_keeps_the_exit_clean_but_a_run_that_skipped_everything_fails() {
        // One of two boards ran: the skip leaves the exit code alone.
        assert_eq!(final_error(&[], 2, 1, Task::Step), None);
        // Nothing ran: the step task had nothing to drive.
        let nothing = final_error(&[], 2, 0, Task::Step).expect("an empty step run fails");
        assert!(nothing.contains("nothing to drive"), "{nothing}");
        // The question only exists for the step task.
        assert_eq!(final_error(&[], 2, 0, Task::Comm), None);

        let failures = vec![format!("zboard: {}", no_stepper_error("zboard", &[]))];
        let failed =
            final_error(&failures, 2, 1, Task::Step).expect("a failed board fails the command");
        assert!(failed.starts_with("1 of 2 board(s) failed"), "{failed}");
        assert!(failed.contains("zboard"), "{failed}");
    }

    // -----------------------------------------------------------------------
    // Real-board motion cases (R5, R11) — declarations and their bodies
    // -----------------------------------------------------------------------

    /// What the R5 hardware case needs of the configured printer: the main MCU,
    /// and `[stepper_x]` with the pins the move drives and the geometry that
    /// sets `step_dist`.
    ///
    /// Kept here so the declaration and the ignored body below cannot drift.
    fn r5_requires() -> crate::hardware_test::Requires {
        crate::hardware_test::Requires::new()
            .mcu()
            .section("stepper_x")
            .option("stepper_x", "step_pin")
            .option("stepper_x", "dir_pin")
            .option("stepper_x", "rotation_distance")
            .option("stepper_x", "microsteps")
    }

    /// What the R11 hardware case needs of the configured printer: the main MCU
    /// and `[stepper_x]` to soak.
    fn r11_requires() -> crate::hardware_test::Requires {
        crate::hardware_test::Requires::new()
            .mcu()
            .section("stepper_x")
    }

    /// The declaration is the gate: it decides whether a case runs or is
    /// reported `HW-IGNORED`. This pins it against synthetic config text parsed
    /// by the real parser, so a declaration that named the wrong section or
    /// option — which would skip the case forever on a printer that could run
    /// it — fails here instead of on hardware.
    #[test]
    fn the_stepper_hardware_cases_declare_what_they_read() {
        use crate::hardware_test::{check, Missing};

        let parse = |text: &str| Config::from_text(text).expect("the fixture parses").0;

        // Everything both cases read: the MCU and `[stepper_x]` with its pins,
        // rotation distance and microsteps.
        let full = parse(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PB1\n\
             rotation_distance: 40\nmicrosteps: 16\n",
        );
        assert_eq!(check(&full, &r5_requires()), Ok(()));
        assert_eq!(check(&full, &r11_requires()), Ok(()));

        // Commented out is absent — the real parser decides, not a string match,
        // so both cases skip rather than run against nothing.
        let commented = parse(
            "[mcu]\nserial: /dev/ttyACM0\n\
             # [stepper_x]\n# step_pin: PA0\n# dir_pin: PB1\n\
             # rotation_distance: 40\n# microsteps: 16\n",
        );
        assert_eq!(
            check(&commented, &r5_requires()),
            Err(vec![Missing::Section("stepper_x".to_string())])
        );
        assert_eq!(
            check(&commented, &r11_requires()),
            Err(vec![Missing::Section("stepper_x".to_string())])
        );

        // An option R5 reads but the config leaves out is its own skip …
        let missing_geometry = parse(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PB1\nrotation_distance: 40\n",
        );
        assert_eq!(
            check(&missing_geometry, &r5_requires()),
            Err(vec![Missing::Option {
                section: "stepper_x".to_string(),
                option: "microsteps".to_string(),
            }])
        );
        // … while R11, which only needs the section, still runs there.
        assert_eq!(check(&missing_geometry, &r11_requires()), Ok(()));

        // The MCU is part of both declarations.
        let no_mcu = parse(
            "[stepper_x]\nstep_pin: PA0\ndir_pin: PB1\n\
             rotation_distance: 40\nmicrosteps: 16\n",
        );
        assert_eq!(
            check(&no_mcu, &r5_requires()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );
        assert_eq!(
            check(&no_mcu, &r11_requires()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );
    }

    /// The one stepper section a case drives, out of the board's, or a message
    /// naming what is missing.
    fn named_stepper(steppers: Vec<StepperSource>, name: &str) -> StepperSource {
        steppers
            .into_iter()
            .find(|section| section.name == name)
            .unwrap_or_else(|| panic!("the board owns no [{name}] stepper to drive"))
    }

    /// R5 — a single-axis move against a real board, reconciled step for step.
    ///
    /// `TESTING.md`'s R5 (lines 52-56) asks for a known move and a read-back:
    /// “电机方向/距离与指令一致，固件步数 = 距离 / `step_dist`”. That is what this
    /// asserts, for the one axis — `[stepper_x]` — through the whole host path
    /// (`Trapq` → `itersolve` → `stepcompress` → `queue_step` → firmware
    /// `stepper_get_position`): the firmware's own count must equal
    /// `round(DISTANCE_MM / step_dist)`, and [`move_one_axis`] fails the case
    /// when it does not.
    ///
    /// **Premise**: the config is assumed correct; this case checks this
    /// repository's motion path, not your config — a mismatch of config, wiring or
    /// firmware is not a bug here.
    ///
    /// # What it does to the machine
    /// It drives **`[stepper_x]` only** (`step_pin`/`dir_pin`) through one move of
    /// `DISTANCE_MM` = 5 mm at `SPEED_MM_S` = 10 mm/s — both inside the 10 mm /
    /// 10 mm/s this case bounds itself to. That is `round(5 / step_dist)` steps:
    /// 400 with the 0.0125 mm/step a 40 mm/rotation × 16-microstep axis gives,
    /// 500 with the 0.01 mm the documented run used. **Make sure the X axis has
    /// at least that ~5 mm of travel before starting**; the move checks no limit
    /// and will step straight into one it is already against.
    ///
    /// # The board it takes over
    /// Like every `stress` run it configures the board's firmware stepper, so a
    /// board currently carrying another configuration — a real printer host's,
    /// say — is `reset` and reconnected first. R5 and R11 configure the same
    /// `[stepper_x]`, so whichever runs second reuses the first's configuration.
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_r5_single_axis_move_matches_the_firmware_step_count() {
        /// R5's move: `DISTANCE_MM` at `SPEED_MM_S`, kept well inside the 10 mm /
        /// 10 mm/s this hardware case allows.
        const DISTANCE_MM: f64 = 5.0;
        const SPEED_MM_S: f64 = 10.0;

        let Some(machine) = crate::hardware_test::acquire(
            "test_r5_single_axis_move_matches_the_firmware_step_count",
            &r5_requires(),
        ) else {
            return; // reported as HW-IGNORED; the test passes without a board
        };
        let mcu_config = resolve_mcu(machine.config(), "mcu").expect("the [mcu] section resolves");
        let pins = build_pin_registry(machine.config()).expect("the pin registry builds");
        let mcu = connect(&mcu_config).await.expect("the board identifies");
        let section = named_stepper(
            find_steppers(machine.config(), "mcu", &mcu, &pins).0,
            "stepper_x",
        );
        move_one_axis(&mcu_config, mcu, &pins, section, DISTANCE_MM, SPEED_MM_S)
            .await
            .expect("R5: the firmware's step count must match the move");
    }

    /// R11 — a short, controlled step soak against a real board.
    ///
    /// `TESTING.md`'s R11 (lines 85-87) asks that continuous stepping leave the
    /// firmware alive — “无 `Stepper too far in past` / `Timer too close`、无丢步、
    /// USB 不掉线”. This is the short form: a fixed, deliberately **conservative**
    /// rate for seconds, instead of the ascending ramp `--task step` runs to the
    /// limit. [`step_soak`] fails the case on a shutdown reason or on a step count
    /// that does not match what was queued (the “无丢步” half), and the case prints
    /// the rate the firmware actually held.
    ///
    /// **Premise**: the config is assumed correct; this case checks this
    /// repository's step path, not your config — a mismatch of config, wiring or
    /// firmware is not a bug here.
    ///
    /// # What it does to the machine
    /// It drives **`[stepper_x]` only**, in one direction (it never reverses).
    /// The rate is `RATE` = 5 000 steps/s — ~68× under the ~339 623 steps/s the
    /// STM32F103 is documented to survive (`TESTING.md`) — for `SECONDS` = 2 s,
    /// i.e. `RATE × SECONDS` = 10 000 steps. The case prints the travel that
    /// implies using the section's own step distance (≈125 mm at a typical
    /// 0.0125 mm/step). **Make sure the axis has that much travel before
    /// starting** (or run it on a bench with the motor disengaged): the soak
    /// neither reverses nor stops at a limit.
    ///
    /// # The board it takes over
    /// As in R5, it configures the board's firmware stepper; a board carrying
    /// another configuration is `reset` and reconnected first, and re-running
    /// this case over the configuration R5 (or this soak) left reuses it.
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_r11_short_soak_holds_a_safe_step_rate() {
        /// The soak's steady rate, in steps per second — a conservative fraction
        /// of the documented ~339 623 steps/s survival line.
        const RATE: f64 = 5_000.0;
        /// How long the soak drives, in seconds.
        const SECONDS: f64 = 2.0;

        let Some(machine) = crate::hardware_test::acquire(
            "test_r11_short_soak_holds_a_safe_step_rate",
            &r11_requires(),
        ) else {
            return; // reported as HW-IGNORED; the test passes without a board
        };
        let mcu_config = resolve_mcu(machine.config(), "mcu").expect("the [mcu] section resolves");
        let pins = build_pin_registry(machine.config()).expect("the pin registry builds");
        let mcu = connect(&mcu_config).await.expect("the board identifies");
        let section = named_stepper(
            find_steppers(machine.config(), "mcu", &mcu, &pins).0,
            "stepper_x",
        );
        let rate = step_soak(&mcu_config, mcu, &pins, section, RATE, SECONDS)
            .await
            .expect("R11: the firmware must hold the safe rate without a shutdown or a lost step");
        println!("R11: held {rate:.0} steps/s for {SECONDS} s with no shutdown");
    }
}
