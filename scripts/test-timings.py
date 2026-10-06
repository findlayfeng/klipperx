#!/usr/bin/env python3
"""Per-test wall time for a `cargo test` run, so "is this normal?" has an answer.

Reads the test binary's stdout on stdin and prints the slowest tests plus a
summary. Two traps it exists to avoid:

* **Build time is not test time.** `cargo test` spends seconds compiling before
  the first "running N tests" banner; attributing that to the first test makes a
  cold run look like a hung test (measured: a 10-16 s "first test"). The clock
  here starts at that banner, and the pre-test time is reported separately.
* **A bounded wait looks exactly like a hang.** Most of this suite's slowest
  tests sleep on the real clock (retry backoff, fixed "wait for X" fallbacks,
  the `M400` wait), so CPU is idle and the process table shows a quiet process
  either way. Only a per-test time says which one it is.

Usage:

    cargo test --workspace -- --test-threads=1 2>&1 | scripts/test-timings.py
    scripts/test-timings.py --threshold 0.5 < run.log

`--test-threads=1` is required: with the default parallel harness the completion
lines interleave across threads and per-test deltas are meaningless (that run is
still useful as a whole-suite wall-clock check).
"""

import argparse
import re
import sys
import time

RUNNING = re.compile(r"^running \d+ tests")
TEST_LINE = re.compile(r"^test (\S+) \.\.\. (ok|FAILED|ignored)")
RESULT = re.compile(r"^test result:")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.0,
        help="flag tests slower than this many seconds (default: 1.0)",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="how many slowest tests to print (default: 20)",
    )
    args = parser.parse_args()

    started = time.time()
    banner_at = None
    last = None
    rows = []
    results = []

    for line in sys.stdin:
        if banner_at is None and RUNNING.match(line):
            banner_at = time.time()
            last = banner_at
            continue
        match = TEST_LINE.match(line)
        if match and last is not None:
            now = time.time()
            rows.append((now - last, match.group(1), match.group(2)))
            last = now
            continue
        if RESULT.match(line):
            results.append(line.strip())

    if banner_at is None:
        print("no 'running N tests' banner seen — is this a cargo test log?", file=sys.stderr)
        return 2

    print(f"build + startup: {banner_at - started:.1f}s (not attributed to any test)")
    for line in results:
        print(line)
    if not rows:
        print("no test result lines parsed")
        return 1

    rows.sort(reverse=True)
    print(f"\nslowest {args.top} (seconds, per test):")
    for elapsed, name, status in rows[: args.top]:
        print(f"{elapsed:8.3f}  {status:8} {name}")

    slow = [(e, n) for e, n, _ in rows if e > args.threshold]
    print(
        f"\ntotal test time: {sum(e for e, _, _ in rows):.1f}s for {len(rows)} tests; "
        f"{len(slow)} slower than {args.threshold:g}s"
    )
    if slow:
        print(
            "each one above is either real work or a bounded wait — check the test body for a\n"
            "sleep/timeout before assuming a hang (see docs/klippy/developer-manual/testing.md)"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
