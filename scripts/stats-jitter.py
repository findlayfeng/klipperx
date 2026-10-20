#!/usr/bin/env python3
"""Device-jitter numbers out of the `stats count= sum= sumsq=` lines of a log.

WHAT IT MEASURES
    An MCU samples, per task round, how many clock ticks that round took, and
    every 5 s sends `stats count=<rounds> sum=<ticks> sumsq=<ticks^2/256>`.
    Those three integers are turned around here into the **per-round task
    duration**: how long the firmware's own work took each time the task ran.
    This is *not* wake-up lateness - a report says nothing about when the host
    asked for the round, only how long the MCU then spent inside it. Host-side
    lateness lives elsewhere (klippy's eventtime/`stats` host decode).

UNITS
    avg, sd : microseconds (us) per task round.
    awake   : percent of the 5 s report window the MCU was inside task rounds.

FORMULAS
    Identical to what the host implements in
    `src/core/klippy/event/stats.rs:136-149`, which are the firmware's own
    accumulation rules from `third_party/klipper/src/basecmd.c:309-343`:

        c     = 1 / (count * CLOCK_FREQ)
        avg   = sum * c
        sd    = c * sqrt(count * sumsq * STATS_SUMSQ_BASE - sum^2)
        awake = sum / CLOCK_FREQ            (seconds inside the 5 s window)

    STATS_SUMSQ_BASE is 256 (`basecmd.c:309`) and the 5 s window is
    `timer_from_us(5000000)` in `stats_update()`. CLOCK_FREQ is the MCU's
    crystal clock, a per-MCU constant - hence the frequency/family handling
    below: reports from different MCUs cannot share one conversion.

TYPICAL CRITERIA (what "healthy" looked like on the F446@180MHz board)
    * avg       : p50 ~5-8 us, p90/p50 < 1.5, max < ~3x p50. A raised avg with
                  flat sd is more work per round; a raised sd with flat avg is
                  irregular timing (interrupt/driver contention).
    * sd        : p50 below ~2 us. sd is the number that moves first.
    * awake     : under ~5%; >20% means the MCU is saturated and every round is
                  already taking long.
    * stability : read the per-bucket table in *ticks* - it is frequency-free,
                  so it answers "stable or not" before the clock is known. A
                  bucket whose avg_ticks drifts by more than ~1.5x against the
                  others is a config/load change inside the log, not noise.

FAMILIES AND FREQUENCY
    A log with more than one MCU (or a config change mid-log) makes the 5 s
    reports split. When the host writes the `[<mcu>]` prefix the split is
    **exact** - one family per MCU, labelled with its name. Logs without the
    prefix (older builds, upstream klippy) fall back to `count`: a faster MCU
    runs more rounds per window. That fallback takes two steps, and neither
    guesses a clock:

      1. activity spikes go first. A report whose `count` is above *3 x the
         median* count is a startup/activity burst, not an MCU; the dropped
         count and the threshold are printed, never dropped silently.
      2. the remaining counts are cut once, at their *largest gap*. The cut
         has to be real: each side needs 10% of the reports (and
         MIN_FAMILY_REPORTS), and the gap itself needs 10% of the median count.
         Otherwise the log is one family and one table is printed - a 2-count
         step inside one board's jitter is not a second MCU.

    k-means - the previous approach - is deliberately not used: seeded at
    `min`/`max`, the centroid at `max` is handed to a handful of startup
    spikes, and the converged cut then separates "8 spikes" from "everything
    else". Both real MCUs stay in one centroid, so their avg/sd get mixed into
    a number that describes neither board.

    The family -> clock mapping is *not* deduced: without `--freq` both
    candidates (`F446@180MHz`, `F103@72MHz`) are printed for every family, so
    the board's crystal decides which row is real. `--freq NAME=HZ` states it
    instead (repeatable), which is also how to describe a board that is
    neither candidate.

USAGE
    Use the repository interpreter, not a bare `python`:

        venv/bin/python scripts/stats-jitter.py ~/printer_data/logs/klippy.log
        venv/bin/python scripts/stats-jitter.py --freq 'F446@180MHz=180000000' klippy.log
        venv/bin/python scripts/stats-jitter.py --interval 30 klippy.log a.log

    Only the standard library is used, so any python3 >= 3.8 can run it; if a
    module is missing the script names the repository venv interpreter instead
    of dying in a traceback (see `.agents/skills/python-tools`).
"""

import argparse
import calendar
import importlib.util
import math
import re
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path

SCRIPT = Path(__file__).name
REPO_VENV_PYTHON = Path(__file__).resolve().parents[1] / "venv" / "bin" / "python"
REQUIRED_MODULES = ("argparse", "calendar", "math", "re", "statistics")

STATS_LINE = re.compile(
    r"(?:\[(?P<mcu>[^\]]+)\] )?stats count=(?P<count>\d+)"
    r" sum=(?P<sum>\d+) sumsq=(?P<sumsq>\d+)"
)
TIMESTAMP = re.compile(
    r"(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d)(?:\.(\d+))?Z"
)

# `basecmd.c:309`: sumsq is accumulated in units of diff^2/256.
SUMSQ_BASE = 256.0
# `stats_update()` reports every timer_from_us(5000000).
REPORT_WINDOW_S = 5.0
# Frequency candidates printed when --freq is not given.
FREQ_CANDIDATES = (("F446@180MHz", 180_000_000.0), ("F103@72MHz", 72_000_000.0))
# Below this many reports a family is too thin to describe.
MIN_FAMILY_REPORTS = 5
# A report whose count is above this multiple of the median count is a startup
# or activity spike rather than an MCU family.
SPIKE_FACTOR = 3.0
# A plausible family holds at least this share of the (spike-free) reports.
MIN_FAMILY_SHARE = 0.10
# The cut only counts as a double peak when it is at least this share of the
# median count: a 2-count step inside one board's own jitter is not two MCUs.
MIN_GAP_SHARE = 0.10


def venv_hint():
    """The repository interpreter to name when this one cannot run the script.

    venv/ is git-ignored and not copied into a worktree, so a worktree checkout
    names the relative path plus how to create it.
    """
    # .absolute(), not .resolve(): the venv interpreter is a symlink to the
    # system python, so resolving would make the two look identical.
    current = Path(sys.executable).absolute()
    if REPO_VENV_PYTHON.exists() and current != REPO_VENV_PYTHON.absolute():
        return f"\nRerun with the repository interpreter: {REPO_VENV_PYTHON} {SCRIPT} ..."
    if not REPO_VENV_PYTHON.exists():
        return (
            "\nThe repository interpreter is venv/bin/python (create it with"
            " `python3 -m venv venv`; see .agents/skills/python-tools)."
        )
    return ""


def require_repo_python():
    """Name the repository interpreter instead of failing in a traceback.

    Standard library only, so this normally finds nothing missing. It exists for
    the machine where a module is absent: then the message says to rerun with
    venv/bin/python rather than leaving the reader with an ImportError.
    """
    missing = [name for name in REQUIRED_MODULES if importlib.util.find_spec(name) is None]
    if not missing:
        return
    sys.exit(f"{SCRIPT}: {sys.executable} is missing {', '.join(missing)}.{venv_hint()}")


def parse_ts(line):
    """ISO-8601 UTC timestamp of a log line, or None if the line has none."""
    match = TIMESTAMP.search(line)
    if not match:
        return None
    year, month, day, hour, minute, second, fraction = match.groups()
    epoch = calendar.timegm(
        (int(year), int(month), int(day), int(hour), int(minute), int(second))
    )
    if fraction:
        epoch += float("0." + fraction)
    return epoch


def load(path):
    """One tuple per `stats` report: (epoch_or_None, count, sum, sumsq, mcu_or_None).

    `mcu` is the name the host now puts in front of the line (`[downstream] stats
    count=…`). Logs written before that prefix existed simply have `None` there,
    which is what makes the count-gap fallback still reachable.
    """
    rows = []
    with open(path, "r", errors="replace") as handle:
        for line in handle:
            match = STATS_LINE.search(line)
            if not match:
                continue
            rows.append(
                (
                    parse_ts(line),
                    int(match.group("count")),
                    int(match.group("sum")),
                    int(match.group("sumsq")),
                    match.group("mcu"),
                )
            )
    return rows


def per_round_ticks(count, total, squares):
    """(avg_ticks, sd_ticks) - per task round, in CLOCK_FREQ-relative ticks."""
    variance = count * squares * SUMSQ_BASE - float(total) ** 2
    return total / count, math.sqrt(max(0.0, variance)) / count


def metrics(count, total, squares, freq):
    """(avg_us, sd_us, awake_percent) for one report, as stats.rs computes it."""
    avg_ticks, sd_ticks = per_round_ticks(count, total, squares)
    return (
        avg_ticks / freq * 1e6,
        sd_ticks / freq * 1e6,
        total / freq / REPORT_WINDOW_S * 100.0,
    )


def percentile(values, fraction):
    """Nearest-rank percentile of an already sorted list."""
    return values[min(len(values) - 1, int(len(values) * fraction))]


def family_label(rows):
    """The MCU's name when the lines carry one, else `count <min>–<max>`."""
    named = {row[4] for row in rows if row[4]}
    if len(named) == 1:
        return named.pop()
    counts = [row[1] for row in rows]
    return f"count {min(counts)}\u2013{max(counts)}"


def split_families(rows):
    """Drop the activity spikes, then split by MCU name or by count gap.

    When the log lines carry the host's `[<mcu>]` prefix, the split is exact:
    one family per MCU, and no guessing from the report rate. Older logs (and
    firmware logs from upstream klippy) have no prefix, so the count gap is
    still the fallback.

    Returns (spike_floor, spikes, candidate, families):
        spike_floor : count above which a report is an activity spike
        spikes      : the reports dropped as spikes
        candidate   : (lower_max, upper_min, gap) of the largest gap, or None
                      when the kept reports have no gap at all
        families    : [(label, rows)] - one entry when no double peak was found
    """
    counts = [row[1] for row in rows]
    spike_floor = SPIKE_FACTOR * statistics.median(counts)
    # The median row is never above SPIKE_FACTOR x median, so `kept` is not empty.
    spikes = [row for row in rows if row[1] > spike_floor]
    ordered = sorted((row for row in rows if row[1] <= spike_floor), key=lambda r: r[1])

    # A name on every kept row is the whole answer: no gap heuristic needed.
    if ordered and all(row[4] for row in ordered):
        names = sorted({row[4] for row in ordered})
        families = [(name, [row for row in ordered if row[4] == name]) for name in names]
        return spike_floor, spikes, None, families

    gap, at = 0, 0
    for index in range(1, len(ordered)):
        if ordered[index][1] - ordered[index - 1][1] > gap:
            gap = ordered[index][1] - ordered[index - 1][1]
            at = index
    candidate = None if len(ordered) < 2 else (ordered[at - 1][1], ordered[at][1], gap)
    single = (spike_floor, spikes, candidate, [(family_label(ordered), ordered)])
    threshold = statistics.median(row[1] for row in ordered)
    thin = max(MIN_FAMILY_REPORTS, MIN_FAMILY_SHARE * len(ordered))
    if candidate is None or at < thin or len(ordered) - at < thin:
        return single
    if gap < MIN_GAP_SHARE * threshold:
        return single
    low, high = ordered[:at], ordered[at:]
    return spike_floor, spikes, candidate, [
        (family_label(low), low),
        (family_label(high), high),
    ]


def family_table(families, freqs):
    """Per family, and per clock inside it: n, avg/sd/awake of the reports."""
    header = (
        f"{'family':18s} {'clock':14s} {'n':>6s} {'avg_p50':>9s} {'avg_p90':>9s}"
        f" {'avg_max':>9s} {'sd_p50':>8s} {'sd_max':>8s}"
        f" {'awake_p50':>10s} {'awake_max':>10s}"
    )
    print("\nper-report numbers (us / %)")
    print(header)
    print("-" * len(header))
    for label, rows in families:
        for clock, freq in freqs:
            converted = [metrics(row[1], row[2], row[3], freq) for row in rows]
            avg = sorted(row[0] for row in converted)
            sd = sorted(row[1] for row in converted)
            awake = sorted(row[2] for row in converted)
            print(
                f"{label:18s} {clock:14s} {len(rows):>6d}"
                f" {statistics.median(avg):>8.1f} {percentile(avg, 0.9):>8.1f}"
                f" {avg[-1]:>8.1f} {statistics.median(sd):>7.1f} {sd[-1]:>7.1f}"
                f" {statistics.median(awake):>9.3f}% {awake[-1]:>9.3f}%"
            )


def bucket_start(epoch, interval_min):
    """Floor an epoch timestamp to an `interval_min`-minute bucket (UTC)."""
    width = interval_min * 60
    return int(epoch) // width * width


def bucket_table(families, interval_min):
    """Per family: how the reports drift, in ticks (no CLOCK_FREQ needed)."""
    print(f"\nstability per {interval_min} min bucket (ticks: avg_ticks = sum/count)")
    header = (
        f"{'family':18s} {'bucket (UTC)':17s} {'n':>5s} {'count_p50':>10s}"
        f" {'avg_ticks_p50':>14s} {'sd_ticks_p50':>13s}"
    )
    for label, rows in families:
        print(f"\n  [{label}, {len(rows)} reports]")
        print("  " + header)
        print("  " + "-" * len(header))
        buckets = {}
        for epoch, count, total, squares, _mcu in rows:
            key = None if epoch is None else bucket_start(epoch, interval_min)
            buckets.setdefault(key, []).append((count, total, squares))
        for key in sorted(buckets, key=lambda k: (k is None, k or 0)):
            group = buckets[key]
            counts = sorted(row[0] for row in group)
            ticks = [per_round_ticks(*row) for row in group]
            avg_ticks = sorted(tick[0] for tick in ticks)
            sd_ticks = sorted(tick[1] for tick in ticks)
            stamp = (
                "no timestamp"
                if key is None
                else datetime.fromtimestamp(key, timezone.utc).strftime("%Y-%m-%d %H:%M")
            )
            print(
                f"  {label:18s} {stamp:17s} {len(group):>5d}"
                f" {statistics.median(counts):>10.0f}"
                f" {statistics.median(avg_ticks):>14.1f}"
                f" {statistics.median(sd_ticks):>13.2f}"
            )


def parse_freq(spec):
    """'NAME=HZ' -> (NAME, hz); exits with a usable message otherwise."""
    name, sep, hertz = spec.partition("=")
    if not sep or not name or not hertz:
        sys.exit(f"--freq needs NAME=HZ (e.g. F446@180MHz=180000000), got {spec!r}")
    try:
        freq = float(hertz)
    except ValueError:
        sys.exit(f"--freq {spec!r}: {hertz!r} is not a number of Hz")
    if freq <= 0:
        sys.exit(f"--freq {spec!r}: frequency must be positive")
    return name, freq


def describe(path, rows, freqs, interval_min, stated_freq):
    print(f"=== {path}  ({len(rows)} stats reports) ===")
    counts = sorted(row[1] for row in rows)
    median = statistics.median(counts)
    print(
        f"reports per window: count min={counts[0]} p50={median:.0f}"
        f" max={counts[-1]}"
    )
    spike_floor, spikes, candidate, families = split_families(rows)
    if spikes:
        print(
            f"spikes: dropped {len(spikes)} of {len(rows)} reports with count >"
            f" {spike_floor:.0f} ({SPIKE_FACTOR:g} x median {median:.0f}), leaving"
            f" count {min(row[1] for row in spikes)}\u2013{max(row[1] for row in spikes)}"
        )
    else:
        print(
            f"spikes: none (no report exceeds {SPIKE_FACTOR:g} x median count"
            f" {spike_floor:.0f})"
        )
    if candidate is None and all(row[4] for row in rows):
        print(
            "families: split by the `[<mcu>]` prefix the host puts on the line -> "
            + " / ".join(f"{label}: {len(group)} reports" for label, group in families)
        )
    elif candidate is None or candidate[2] == 0:
        print("families: no gap between the counts -> no double peak detected -> one family")
    elif len(families) == 1:
        print(
            f"families: largest gap {candidate[0]} -> {candidate[1]}"
            f" (gap {candidate[2]}) is too shallow to be a second MCU"
            " -> no double peak detected -> one family"
        )
    else:
        print(
            f"families: cut at the largest gap {candidate[0]} -> {candidate[1]}"
            f" (gap {candidate[2]}) -> "
            + " / ".join(f"{label}: {len(group)} reports" for label, group in families)
        )
    if not stated_freq:
        candidates = ", ".join(name for name, _ in FREQ_CANDIDATES)
        print(
            "clock: not stated, so every family is shown for both candidates"
            f" ({candidates});"
        )
        print("       family -> clock mapping unknown: the board's crystal decides")
    family_table(families, freqs)
    bucket_table(families, interval_min)


def main(argv=None):
    require_repo_python()
    parser = argparse.ArgumentParser(
        description=(
            "Turn the `stats count= sum= sumsq=` reports of a klippy.log into"
            " per-round task duration (avg/sd/awake) per MCU family, plus a"
            " per-bucket stability table. Measures how long each firmware task"
            " round took, not wake-up lateness."
        ),
        epilog=(
            "examples:\n"
            "  venv/bin/python scripts/stats-jitter.py ~/printer_data/logs/klippy.log\n"
            "  venv/bin/python scripts/stats-jitter.py --freq F446@180MHz=180000000 klippy.log\n"
            "  venv/bin/python scripts/stats-jitter.py --interval 30 klippy.log\n"
            "\n"
            "families: activity spikes go first (count > 3 x the median count), then the\n"
            "remaining counts are cut once at their largest gap; if either side would be\n"
            "under 10% of the reports, or the gap under 10% of the median count, there\n"
            "is no double peak and one table is printed.\n"
            "k-means is not used: seeded at min/max it gives one centroid to a few startup\n"
            "spikes, so it splits spikes-vs-everything instead of MCU-vs-MCU.\n"
            "units: avg/sd in us per task round, awake in % of the 5 s report window.\n"
            "criteria: avg p50 ~5-8 us with p90/p50 < 1.5, sd p50 < ~2 us, awake < ~5%,\n"
            "and no bucket in the ticks table drifting by more than ~1.5x.\n"
            "formulas: src/core/klippy/event/stats.rs:136-149 and\n"
            "third_party/klipper/src/basecmd.c:309-343."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "logs",
        nargs="+",
        metavar="LOG",
        help="klippy.log to read (several accepted; also works on a saved copy)",
    )
    parser.add_argument(
        "--freq",
        action="append",
        metavar="NAME=HZ",
        help=(
            "clock frequency of the MCU, repeatable; stating it fixes which rows are"
            " results (default: both candidates F446@180MHz and F103@72MHz per family,"
            " family -> clock mapping then unknown)"
        ),
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=10,
        metavar="MIN",
        help="stability table bucket width in minutes (default: 10)",
    )
    args = parser.parse_args(argv)
    if args.interval <= 0:
        parser.error("--interval must be a positive number of minutes")

    freqs = (
        tuple(parse_freq(spec) for spec in args.freq) if args.freq else FREQ_CANDIDATES
    )
    empty = []
    for path in args.logs:
        try:
            rows = load(path)
        except OSError as error:
            sys.exit(f"cannot read {path}: {error}")
        if not rows:
            empty.append(path)
            continue
        describe(path, rows, freqs, args.interval, stated_freq=bool(args.freq))
    if empty:
        print(
            f"\nno `stats count= sum= sumsq=` lines in: {', '.join(empty)}"
            " (this reads the MCU stats reports, not host log lines)",
            file=sys.stderr,
        )
    return 1 if empty and len(empty) == len(args.logs) else 0


if __name__ == "__main__":
    sys.exit(main())
