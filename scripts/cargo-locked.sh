#!/bin/bash
#
# cargo-locked.sh — run cargo, but say out loud when another cargo holds the
# target directory's build lock.
#
# Why: cargo serializes builds that share a target directory by blocking on a
# lock file, and it does so silently. A queued cargo has an idle CPU and a quiet
# process table, exactly like a hung test — that has already been misread once
# here (a cold build's seconds attributed to the first test). This wrapper probes
# the lock first, so a wait reads as a wait.
#
# What cargo locks, measured against cargo 1.97.1 here (`strace -f -e
# trace=flock,openat cargo build`, and probing a build that was already running
# with `flock -n`):
#
#   <target-dir>/<profile-dir>/.cargo-lock            LOCK_SH  read-only ops
#   <target-dir>/<profile-dir>/.cargo-build-lock      LOCK_EX  the build queue
#   <target-dir>/<profile-dir>/.cargo-artifact-lock   LOCK_EX  artifact writes
#
# `<profile-dir>` is where the artifacts go: `debug` for the dev and test
# profiles, `release` for release and bench. `--release` locks only that
# profile's files, and `CARGO_TARGET_DIR` moves the whole tree. Cargo uses
# flock(2), not fcntl, so `flock` on the same file is the same test cargo itself
# makes: the strace of a second cargo shows `flock(fd, LOCK_EX|LOCK_NB)`
# returning EAGAIN on `.cargo-build-lock` and then blocking in
# `flock(fd, LOCK_EX)` on it. That file is what this waits on.
#
# Two builds contend only when the target directory *and* the profile match —
# measured: a second `cargo build -q` in the same profile waited out the
# remaining 17.1 s of a 25 s build, while a `--release` build finished in 16 ms
# under a held debug lock. And only the build is serialized: once the artifacts
# are up to date cargo releases the lock before running the test binaries
# (measured: all three files free, test executable running).
#
# Usage:
#   scripts/cargo-locked.sh test --workspace
#   scripts/cargo-locked.sh build --release
#   scripts/cargo-locked.sh cargo test -p klipperx --lib nonexistent_filter
#   CARGO_TARGET_DIR=/tmp/t scripts/cargo-locked.sh test -p klipperx --lib
#
# A leading `cargo` is optional, so a command line that already starts with
# `cargo` can be prefixed with the wrapper as is. Everything else is passed to
# cargo untouched, and the script `exec`s cargo, so cargo's output and exit
# status stay its own — including a non-zero one.
#
# Not followed: `--target <triple>` (a cross build puts that profile's artifacts
# and lock under `<target-dir>/<triple>/`) and `--manifest-path <other>`. Either
# one can hide a lock from this probe; the wrapper then stays quiet and cargo
# queues exactly as it always has, which is the safe way to be wrong.
#
# No timeout of its own: cargo has none, and a bounded default would turn a
# legitimately long build into a failure. A wait prints a heartbeat every 30 s —
# elapsed time plus which processes have the lock file open — so a long queue is
# never silent and can be told from a hang.
#
set -euo pipefail

heartbeat_s=30

# The profile directory cargo would use for this command line: cargo's own
# mapping (Profile::dir_name) — dev and test share `debug`, release and bench
# share `release`, a user-defined profile is its own name.
profile_dir_of() {
    case $1 in
        ""|dev|test) printf 'debug' ;;
        release|bench) printf 'release' ;;
        *) printf '%s' "$1" ;;
    esac
}

# Wall time between two `date +%s%N` stamps, to one decimal. Nanoseconds keep
# the arithmetic integer, so no locale decimal point gets in the way.
fmt_elapsed() {
    local ns
    ns=$(($2 - $1))
    printf '%d.%ds' $((ns / 1000000000)) $(((ns % 1000000000) / 100000000))
}

# Which processes have the lock file open, from /proc. The probe above only says
# "held"; this says by what. A queued cargo has the file open too, so waiters are
# listed as well — the point is telling a cargo queue from a stray process.
list_open_holders() {
    local fd pid cmd
    for fd in /proc/[0-9]*/fd/*; do
        [ -e "$fd" ] || continue
        [ "$(readlink "$fd" 2>/dev/null)" = "$lock_real" ] || continue
        pid=${fd#/proc/}
        pid=${pid%%/*}
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
        printf 'cargo-locked:   open in pid %s: %s\n' "$pid" "${cmd% }" >&2
    done
}

# Wait until the build lock can be taken and let it go again at once. Cargo
# takes the lock itself a moment later; this process holding it across the `exec`
# would deadlock against cargo's own blocking lock request.
wait_for_lock() {
    local start_ns last_note probe now_ns
    start_ns=$(date +%s%N)
    last_note=$start_ns
    while :; do
        probe=0
        flock -w 1 "$lock_file" -c true 2>/dev/null || probe=$?
        if [ "$probe" = 0 ]; then
            now_ns=$(date +%s%N)
            printf 'cargo-locked: acquired after %s\n' "$(fmt_elapsed "$start_ns" "$now_ns")" >&2
            return 0
        fi
        if [ "$probe" != 1 ]; then
            printf 'cargo-locked: cannot probe %s (flock exited %s); running cargo anyway\n' \
                "$lock_file" "$probe" >&2
            return 1
        fi
        now_ns=$(date +%s%N)
        if [ $(((now_ns - last_note) / 1000000000)) -ge "$heartbeat_s" ]; then
            printf 'cargo-locked: still waiting after %s\n' "$(fmt_elapsed "$start_ns" "$now_ns")" >&2
            list_open_holders
            last_note=$now_ns
        fi
    done
}

cargo_args=("$@")
case ${cargo_args[0]:-} in
    cargo|*/cargo) cargo_args=("${cargo_args[@]:1}") ;;
esac

# Which profile, and which target directory. `--target-dir` and the
# `CARGO_TARGET_DIR` environment variable override the workspace's own `target/`
# (a relative value is relative to the current directory for cargo, and this
# script never changes directory, so it stays relative here too).
profile_dir=debug
target_dir=${CARGO_TARGET_DIR:-}
i=0
while [ "$i" -lt "${#cargo_args[@]}" ]; do
    arg=${cargo_args[$i]}
    case $arg in
        --) break ;;
        -r|--release) profile_dir=release ;;
        --profile)
            i=$((i + 1))
            profile_dir=$(profile_dir_of "${cargo_args[$i]:-}")
            ;;
        --profile=*) profile_dir=$(profile_dir_of "${arg#--profile=}") ;;
        --target-dir)
            i=$((i + 1))
            target_dir=${cargo_args[$i]:-}
            ;;
        --target-dir=*) target_dir=${arg#--target-dir=} ;;
    esac
    i=$((i + 1))
done

if [ -z "$target_dir" ]; then
    # The workspace root, from cargo itself: the wrapper runs from wherever the
    # user is — a member crate directory, say — while `target/` belongs to the
    # workspace root. Outside a project, hand over and let cargo say so.
    if ! manifest=$(cargo locate-project --workspace --message-format plain 2>/dev/null); then
        exec cargo "${cargo_args[@]}"
    fi
    target_dir=$(dirname "$manifest")/target
fi

lock_file=$target_dir/$profile_dir/.cargo-build-lock
if [ ! -e "$lock_file" ]; then
    # No lock file means nobody can hold it: a first build, target directory and
    # all, has nothing to wait for. Probing would create the file (flock(1) opens
    # with O_CREAT), so this existence check is also what keeps the wrapper from
    # leaving one behind.
    exec cargo "${cargo_args[@]}"
fi

# `flock -n` takes LOCK_EX without blocking: cargo's own first attempt, so a
# zero status means the queue is empty. Status 1 is EAGAIN, the lock is held;
# anything else is a probe that could not even open the file, which is not
# something to block on.
probe=0
flock -n "$lock_file" -c true 2>/dev/null || probe=$?
case $probe in
    0) exec cargo "${cargo_args[@]}" ;;
    1) ;;
    *)
        printf 'cargo-locked: cannot probe %s (flock exited %s); running cargo anyway\n' \
            "$lock_file" "$probe" >&2
        exec cargo "${cargo_args[@]}"
        ;;
esac

lock_real=$(readlink -f "$lock_file")
printf 'cargo-locked: another cargo holds %s; waiting (idle CPU here is expected, not a hang)\n' \
    "$lock_file" >&2
list_open_holders
wait_for_lock || true

exec cargo "${cargo_args[@]}"
