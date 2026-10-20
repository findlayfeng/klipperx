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
    reports split by `count`: a faster MCU runs more rounds per window. By
    default the report counts are clustered into two families (`kmeans2` on
    `count`, split at the midpoint; a unimodal log stays one family). The
    clock frequency itself is *not* guessed: two candidates are printed
    (`F446@180MHz`, `F103@72MHz`) so the reader picks the one matching the
    board. `--freq NAME=HZ` states it instead (repeatable), which is also how
    to describe a board that is neither candidate.

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

STATS_LINE = re.compile(r"stats count=(\d+) sum=(\d+) sumsq=(\d+)")
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
    """One tuple per `stats` report: (epoch_or_None, count, sum, sumsq)."""
    rows = []
    with open(path, "r", errors="replace") as handle:
        for line in handle:
            match = STATS_LINE.search(line)
            if not match:
                continue
            rows.append(
                (
                    parse_ts(line),
                    int(match.group(1)),
                    int(match.group(2)),
                    int(match.group(3)),
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


def kmeans2(values):
    """Two 1-D centroids, seeded at min/max; None if the data will not split."""
    low, high = min(values), max(values)
    left, right = low, high
    for _ in range(50):
        near_left = [value for value in values if abs(value - left) <= abs(value - right)]
        near_right = [value for value in values if abs(value - left) > abs(value - right)]
        if not near_left or not near_right:
            return None
        new_left = sum(near_left) / len(near_left)
        new_right = sum(near_right) / len(near_right)
        if (new_left, new_right) == (left, right):
            break
        left, right = new_left, new_right
    return sorted((left, right))


def split_families(rows):
    """[(label, rows)] - one family when the counts are unimodal, else two."""
    counts = [row[1] for row in rows]
    if max(counts) - min(counts) <= 10:
        return [("all reports", rows)]
    centroids = kmeans2(counts)
    if centroids is None:
        return [("all reports", rows)]
    threshold = (centroids[0] + centroids[1]) / 2.0
    low = [row for row in rows if row[1] < threshold]
    high = [row for row in rows if row[1] >= threshold]
    if len(low) < MIN_FAMILY_REPORTS or len(high) < MIN_FAMILY_REPORTS:
        return [("all reports", rows)]
    return [
        (f"count < {threshold:.0f}", low),
        (f"count >= {threshold:.0f}", high),
    ]


def family_table(families, freqs):
    """Per family and per candidate clock: n, avg/sd/awake of the reports."""
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
            converted = [metrics(c, s, q, freq) for _, c, s, q in rows]
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
        for epoch, count, total, squares in rows:
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


def describe(path, rows, freqs, interval_min):
    print(f"=== {path}  ({len(rows)} stats reports) ===")
    counts = sorted(row[1] for row in rows)
    print(
        f"reports per window: count min={counts[0]} p50={statistics.median(counts):.0f}"
        f" max={counts[-1]}"
    )
    families = split_families(rows)
    if len(families) == 1:
        print("report counts are unimodal -> one family")
    else:
        print(
            "report counts are bimodal -> "
            + " / ".join(f"{label}: {len(group)} reports" for label, group in families)
        )
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
            "clock frequency of the MCU, repeatable"
            " (default: print the candidates F446@180MHz and F103@72MHz)"
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
        describe(path, rows, freqs, args.interval)
    if empty:
        print(
            f"\nno `stats count= sum= sumsq=` lines in: {', '.join(empty)}"
            " (this reads the MCU stats reports, not host log lines)",
            file=sys.stderr,
        )
    return 1 if empty and len(empty) == len(args.logs) else 0


if __name__ == "__main__":
    sys.exit(main())
