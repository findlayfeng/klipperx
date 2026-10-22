#!/bin/sh
# Run cargo test with the whole output saved, then report failures **by name**.
#
# Why: grepping `^test result:` alone loses the failing test's name, and a
# failure that is seen once and not captured cannot be chased afterwards (two
# single-run failures in this repo went unidentified that way).
#
#   scripts/test-log.sh -p klipperx --lib
#   scripts/test-log.sh -l gate --workspace
#   scripts/test-log.sh -l pending -p klipperx --lib mcu::pending -- --test-threads=1
#
# Every run lands in `target/tmp/test-logs/<timestamp>-<label>.log` (inside the
# ignored target tree, so nothing leaks into the repository or into `/tmp`,
# which a reboot takes away), and the newest 20 are kept. The cargo exit code is
# passed through unchanged.
#
# Environment:
#   TEST_LOG_DIR   where the logs go (default `target/tmp/test-logs`)
#   TEST_LOG_KEEP  how many to keep (default 20)

set -u

log_dir=${TEST_LOG_DIR:-target/tmp/test-logs}
keep=${TEST_LOG_KEEP:-20}
label=test

while [ $# -gt 0 ]; do
    case $1 in
    -l)
        label=$2
        shift 2
        ;;
    --)
        shift
        break
        ;;
    *)
        break
        ;;
    esac
done

mkdir -p "$log_dir" || exit 1
stamp=$(date +%Y%m%d-%H%M%S)
log="$log_dir/$stamp-$label.log"

cargo test "$@" >"$log" 2>&1
rc=$?

# Failures by name, then the tail of the output: the order that made the name
# reachable at all.
if [ "$rc" -ne 0 ]; then
    if grep -qE '^test .* FAILED' "$log"; then
        echo "failed tests:"
        grep -E '^test .* FAILED' "$log"
    fi
    # The panic text sits under `---- <name> stdout ----`.
    if grep -q '^failures:$' "$log"; then
        sed -n '/^failures:$/,$p' "$log" | head -60
    else
        # No test-name lines: this is a build/compile failure or a bad filter.
        # The compiler's diagnostics are the last thing worth reading.
        echo "no test failures in the log (build error?); tail:"
        tail -n 25 "$log"
    fi
else
    grep -E '^test result:' "$log"
fi

echo "full output: $log"

# Keep the directory bounded: oldest first, everything past $keep goes.
if [ "$keep" -gt 0 ]; then
    ls -1t "$log_dir"/*.log 2>/dev/null | sed -n "$((keep + 1)),\$p" |
        while IFS= read -r old; do rm -f "$old"; done
fi

exit "$rc"
