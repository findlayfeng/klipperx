#!/usr/bin/env python3
"""Per-thread scheduler statistics of a live process: who waits on the run queue.

WHAT IT MEASURES
    For every thread of `--pid` it reads `/proc/<pid>/task/<tid>/schedstat` and
    `/proc/<pid>/task/<tid>/sched` **twice**, `--interval` seconds apart, and
    reports the difference. Only differences are meaningful: both files hold
    lifetime counters since thread start, so a single read says nothing about
    the run being measured.

    The interesting column is `mean_wait_us`: how long, on average, the thread
    sat runnable-but-not-running before each time it was scheduled (`sched_in`).
    That wait is the OS-level share of "thread switching jitter" - the part the
    kernel adds on top of whatever the thread itself does. It is not the
    thread's own work time (`cpu_ms`) and not its wake-up latency as a device
    sees it.

UNITS
    sched_in   : number of times threads were put on a CPU during the interval.
    preempt    : of those, involuntary switches (time slice / higher-prio preempt).
    cpu_ms     : CPU milliseconds the thread used during the interval
                 (`schedstat` field 1, nanoseconds in the file).
    mean_wait_us: run-queue wait per scheduling event, microseconds
                 (`schedstat` field 2 delta / sched_in delta; the file is in ns).

TYPICAL CRITERIA
    * A thread that is mostly idle: mean_wait_us in the low single digits and
      cpu_ms well under the interval in ms.
    * A busy thread on an unloaded machine: mean_wait_us stays under ~tens of
      us. Climbing into hundreds of us to ms-means it is being starved - look
      at what else on the box is runnable, not at this process.
    * preempt ~ sched_in means the scheduler keeps taking the CPU away
      (many small wakeups or a competing real-time load), which shows up as
      jitter in whatever this thread is responsible for timing.

WHY DIFFERENCES STILL WORK WITH `kernel.sched_schedstats=0`
    That sysctl (and its default on this machine, 0) is about the tracepoints
    and the `/proc/<pid>/sched` statistics; the run-queue wait in
    `/proc/<pid>/task/<tid>/schedstat` is still accumulated and still moves
    (measured: a thread's wait field grows between two reads while the sysctl
    reads 0). So the delta is valid without touching the sysctl.

WHAT IS NOT AVAILABLE
    This machine's `/proc/<tid>/sched` has no `sched_info` block (the
    CONFIG_SCHED_INFO fields), so the worst single wait - `wait_max`, often more
    telling than the mean - cannot be read here. Only the mean wait, the
    involuntary switch count and the exec runtime are on offer.

USAGE
    Use the repository interpreter, not a bare `python`:

        venv/bin/python scripts/kx-sched.py --pid $(pgrep -f klipperx | head -1)
        venv/bin/python scripts/kx-sched.py --pid 1234 --interval 10 --top 5

    Only the standard library is used, so any python3 >= 3.8 can run it; if a
    module is missing the script names the repository venv interpreter instead
    of dying in a traceback (see `.agents/skills/python-tools`).
"""

import argparse
import importlib.util
import os
import sys
import time
from pathlib import Path

SCRIPT = Path(__file__).name
REPO_VENV_PYTHON = Path(__file__).resolve().parents[1] / "venv" / "bin" / "python"
REQUIRED_MODULES = ("argparse", "os", "time")


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


class ProcessGone(Exception):
    """The pid disappeared (or was never there) while taking a snapshot."""


def snapshot(pid):
    """{tid: counters} for every thread of `pid`, read at one moment."""
    base = f"/proc/{pid}/task"
    try:
        tids = os.listdir(base)
    except (FileNotFoundError, NotADirectoryError):
        raise ProcessGone(f"/proc/{pid}/task does not exist") from None
    except PermissionError:
        raise ProcessGone(f"no permission to read /proc/{pid}/task") from None

    threads = {}
    for tid in tids:
        directory = f"{base}/{tid}"
        try:
            with open(f"{directory}/comm") as handle:
                comm = handle.read().strip()
            with open(f"{directory}/schedstat") as handle:
                runtime_ns, wait_ns, sched_in = (int(x) for x in handle.read().split())
            involuntary = None
            with open(f"{directory}/sched") as handle:
                for line in handle:
                    if line.startswith("nr_involuntary_switches"):
                        involuntary = int(line.split(":")[1])
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            # Thread exited between listdir() and the open calls.
            continue
        threads[tid] = {
            "comm": comm,
            "runtime_ns": runtime_ns,
            "wait_ns": wait_ns,
            "sched_in": sched_in,
            "involuntary": involuntary,
        }
    if not threads:
        raise ProcessGone(f"/proc/{pid}/task holds no readable thread")
    return threads


def deltas(before, after):
    """[(comm, tid, sched_in, preempt, cpu_ms, mean_wait_us, wait_us)] per thread."""
    rows = []
    for tid, current in after.items():
        previous = before.get(tid)
        if previous is None:
            continue  # thread started after the first snapshot
        sched_in = current["sched_in"] - previous["sched_in"]
        wait_us = (current["wait_ns"] - previous["wait_ns"]) / 1e3
        preempt = (current["involuntary"] or 0) - (previous["involuntary"] or 0)
        cpu_ms = (current["runtime_ns"] - previous["runtime_ns"]) / 1e6
        mean_wait_us = wait_us / sched_in if sched_in > 0 else 0.0
        rows.append(
            (current["comm"], tid, sched_in, preempt, cpu_ms, mean_wait_us, wait_us)
        )
    return rows


def report(pid, interval, rows, top):
    active = [row for row in rows if row[2] > 0 or row[4] > 0]
    active.sort(key=lambda row: (-row[2], -row[4]))
    total_sched_in = sum(row[2] for row in active)
    total_cpu_ms = sum(row[4] for row in active)
    print(
        f"pid={pid} interval={interval}s threads_with_activity={len(active)}"
        f" (total {len(rows)} threads stayed alive)"
    )
    print(
        f"totals: sched_in={total_sched_in} cpu_ms={total_cpu_ms:.1f}"
        f" ({total_cpu_ms / (interval * 1000.0) * 100.0:.0f}% of one CPU"
        f" over {interval}s)"
    )
    header = (
        f"{'thread':22s} {'tid':>8s} {'sched_in':>9s} {'preempt':>8s}"
        f" {'cpu_ms':>8s} {'mean_wait_us':>13s} {'wait_ms':>8s}"
    )
    print(header)
    print("-" * len(header))
    shown = active if top <= 0 else active[:top]
    for comm, tid, sched_in, preempt, cpu_ms, mean_wait_us, wait_us in shown:
        print(
            f"{comm[:22]:22s} {tid:>8s} {sched_in:>9d} {preempt:>8d}"
            f" {cpu_ms:>8.1f} {mean_wait_us:>13.2f} {wait_us / 1e3:>8.3f}"
        )
    if len(shown) < len(active):
        hidden = active[len(shown):]
        print(
            f"... {len(hidden)} more threads below the top {len(shown)}"
            f" (--top 0 lists all; busiest hidden: sched_in={hidden[0][2]}"
            f" cpu_ms={hidden[0][4]:.1f})"
        )
    if not active:
        print("no thread was scheduled during the interval - is the process idle?")


def main(argv=None):
    require_repo_python()
    parser = argparse.ArgumentParser(
        description=(
            "Per-thread run-queue statistics of a live process, as a difference"
            " between two snapshots of /proc/<pid>/task/*/{schedstat,sched}."
            " mean_wait_us is the average run-queue wait before each time the"
            " thread was scheduled; it is the OS-level share of switching"
            " jitter, not the thread's own compute time."
        ),
        epilog=(
            "examples:\n"
            "  venv/bin/python scripts/kx-sched.py --pid $(pgrep -f klipperx | head -1)\n"
            "  venv/bin/python scripts/kx-sched.py --pid 1234 --interval 10 --top 5\n"
            "  venv/bin/python scripts/kx-sched.py --pid 1234 --top 0\n"
            "\n"
            "units: sched_in/preempt are event counts, cpu_ms and wait_ms are\n"
            "milliseconds over the interval, mean_wait_us is microseconds per\n"
            "scheduling event.\n"
            "criteria: an idle thread shows single-digit mean_wait_us; a busy thread\n"
            "on a quiet machine stays under ~tens of us; hundreds of us to ms means\n"
            "it is being starved (look at what else is runnable). preempt close to\n"
            "sched_in means the scheduler keeps taking the CPU away.\n"
            "notes: works with kernel.sched_schedstats=0 (the schedstat wait field\n"
            "still moves, so differences are valid); wait_max is not available -\n"
            "this kernel's /proc/<tid>/sched has no sched_info block."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--pid",
        type=int,
        required=True,
        metavar="PID",
        help="process to sample (e.g. a running klipperx host)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="how long to wait between the two snapshots (default: 30)",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        metavar="N",
        help="list only the N busiest threads, 0 for all (default: 20)",
    )
    args = parser.parse_args(argv)

    if args.pid <= 0:
        parser.error("--pid must be a positive process id")
    if args.interval <= 0:
        parser.error("--interval must be a positive number of seconds")
    if args.top < 0:
        parser.error("--top must be >= 0 (0 lists every thread)")

    try:
        before = snapshot(args.pid)
        time.sleep(args.interval)
        after = snapshot(args.pid)
    except ProcessGone as error:
        sys.exit(f"pid {args.pid}: {error}")

    report(args.pid, args.interval, deltas(before, after), args.top)
    return 0


if __name__ == "__main__":
    sys.exit(main())
