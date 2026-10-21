#!/usr/bin/env bash
# host-stability.sh - one bounded battery for "is this host steady?" on a card-sized
# machine (RK3328 / RK3566 / CM4 class), producing a report you can paste back.
#
# What it does, in order (each stage is bounded, nothing runs forever):
#   0. fingerprint    CPU model, cores, cgroup CPU quota, memory, kernel, build
#   A. idle soak      start a host with no hardware at all, sample for --soak seconds
#   B. api latency    100 sequential + 8-way concurrent requests, percentiles
#   C. restart storm  N x RESTART, watching thread count and RSS
#   D. jitter         per-thread run-queue wait while B is running (scripts/kx-sched.py)
#   E. real board     only with --config: read-only health of a real session
#
# Usage:
#   scripts/host-stability.sh                       # default: full battery, no real board
#   scripts/host-stability.sh --quick               # shorter soak/storm (CI-ish)
#   scripts/host-stability.sh --config ~/printer_data/config/printer.cfg   # add stage E
#   scripts/host-stability.sh --out /tmp/kx-hs      # where the report goes
#
# Exit code: 0 when every hard check passed, 1 otherwise (advisory checks only warn).
set -u

REPO=$(cd "$(dirname "$0")/.." && pwd)
BIN=${BIN:-$REPO/target/release/klipperx}
OUT=${OUT:-$PWD/host-stability-$(date +%Y%m%d-%H%M%S)}
PORT=${PORT:-7150}
SOAK=60
STORM=60
CONFIG=""
QUICK=0

while [ $# -gt 0 ]; do
    case "$1" in
        --quick) QUICK=1; SOAK=20; STORM=15; shift ;;
        --soak) SOAK=$2; shift 2 ;;
        --storm) STORM=$2; shift 2 ;;
        --config) CONFIG=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --bin) BIN=$2; shift 2 ;;
        --port) PORT=$2; shift 2 ;;
        -h|--help) sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

# A host left behind by an aborted run would answer on our port and make every
# stage measure the wrong process. Take the first free one instead of insisting.
pick_port() {
    local candidate=$1 tries=0
    while [ $tries -lt 20 ]; do
        if "$PY" -c "import socket,sys; s=socket.socket(); s.settimeout(0.2); sys.exit(1 if s.connect_ex(('127.0.0.1', $candidate)) == 0 else 0)"; then
            echo "$candidate"
            return 0
        fi
        candidate=$((candidate + 1)); tries=$((tries + 1))
    done
    return 1
}

PY=$REPO/venv/bin/python
[ -x "$PY" ] || PY=$(command -v python3 || true)
if [ -z "$PY" ]; then
    echo "no python3 found; the jitter and JSON steps need it" >&2
    exit 2
fi

WANTED_PORT=$PORT
PORT=$(pick_port "$PORT") || { echo "no free port near $WANTED_PORT" >&2; exit 2; }
[ "$PORT" = "$WANTED_PORT" ] || echo "port $WANTED_PORT is busy; using $PORT" >&2

mkdir -p "$OUT" || exit 2
REPORT=$OUT/report.md
RAW=$OUT/raw
mkdir -p "$RAW"
: > "$REPORT"
FAILED=0
WARNED=0
STARTED=""
start_host_pid() { STARTED="$STARTED $1"; }
cleanup_hosts() {
    local pid
    for pid in $STARTED; do
        kill -TERM "$pid" 2>/dev/null
    done
}
trap cleanup_hosts EXIT HUP INT TERM

say() { printf '%s\n' "$*" | tee -a "$REPORT" >/dev/null; }
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
pass() { say "- PASS  $*"; log "PASS  $*"; }
fail() { say "- **FAIL**  $*"; log "FAIL  $*"; FAILED=$((FAILED + 1)); }
advisory() { say "- warn  $*"; WARNED=$((WARNED + 1)); log "warn  $*"; }

say "# host-stability report"
say ""
say "\`$(date -u +%Y-%m-%dT%H:%M:%SZ)\` on \`$(hostname)\`"
say ""

# ---------------------------------------------------------------- 0 fingerprint
log "stage 0: fingerprint"
{
    echo "## 0. machine"
    echo ""
    echo "| item | value |"
    echo "|---|---|"
    echo "| kernel | \`$(uname -sr)\` |"
    model=$(awk -F: '/^(Model|model name|Hardware)/ {gsub(/^ +| +$/, "", $2); print $2; exit}' /proc/cpuinfo)
    echo "| cpu | \`${model:-unknown}\` |"
    echo "| cores (affinity) | $(nproc) |"
    echo "| memory | $(awk '/MemTotal/{printf "%.0f MiB", $2/1024}' /proc/meminfo) |"
    quota=$(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo "n/a")
    echo "| cgroup cpu.max | \`$quota\` |"
    echo "| loadavg | \`$(cut -d' ' -f1-3 /proc/loadavg)\` |"
    echo "| binary | \`$BIN\` |"
    if [ -x "$BIN" ]; then
        echo "| binary size | $(stat -c %s "$BIN") bytes |"
    else
        echo "| binary | **missing** |"
    fi
    if command -v git >/dev/null && git -C "$REPO" rev-parse HEAD >/dev/null 2>&1; then
        echo "| commit | \`$(git -C "$REPO" rev-parse --short HEAD)\` |"
    fi
    echo ""
} >> "$REPORT"

if [ ! -x "$BIN" ]; then
    say "- **FAIL**  no binary at \`$BIN\`. Build it first: \`cargo build --release\`"
    say ""
    echo "wrote $REPORT"
    exit 1
fi
if ! grep -q 'release' <<<"$(dirname "$BIN")"; then
    advisory "this looks like a debug build (\`$BIN\`); timings will not compare with a release run"
fi

# ---------------------------------------------------------------- helpers
wait_ready() {
    # $1 = pid; up to ~30 s. The host is ready when `info` answers `ready`.
    local pid=$1 i
    for i in $(seq 60); do
        kill -0 "$pid" 2>/dev/null || return 1
        if "$BIN" api -a "tcp:127.0.0.1:$PORT" info '{}' 2>/dev/null | grep -q '"state": "ready"'; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

threads_of() { ls "/proc/$1/task" 2>/dev/null | wc -l; }
rss_of() { awk '/VmRSS/{print $2}' "/proc/$1/status" 2>/dev/null; }
query() { "$BIN" api -a "tcp:127.0.0.1:$PORT" objects/query "$1" 2>/dev/null; }
query_port() { "$BIN" api -a "tcp:127.0.0.1:$1" objects/query "$2" 2>/dev/null; }

# One line of numbers from `system_stats`: busy_avg lateness_avg over_threshold dropped_logs.
# Exits non-zero (rather than printing blanks) when a key is missing, so a binary
# that predates `system_stats` cannot look like a pass: an unknown object answers
# with an empty object, which is exactly the shape that would otherwise be
# mistaken for "nothing wrong".
read_dispatch() {
    query '{"objects":{"system_stats":["dispatch","dropped_logs"]}}' | "$PY" -c '
import json, sys
status = json.load(sys.stdin)["status"]["system_stats"]
rounds = status["dispatch"]
print(
    "%.6f %.6f %d %d"
    % (
        rounds["busy_avg_s"],
        rounds["lateness_avg_s"],
        rounds["over_threshold"],
        status["dropped_logs"],
    )
)' 2>/dev/null
}

start_host() {
    # $1 = config path; echoes the pid.
    "$BIN" klippy -a "tcp:127.0.0.1:$PORT" --logfile "$RAW/klippy.log" "$1" \
        >"$RAW/klippy.out" 2>&1 &
    start_host_pid $!
    echo $!
}

# ---------------------------------------------------------------- A idle soak
log "stage A: idle soak (${SOAK}s) on a host with no hardware"
say "## A. idle soak (${SOAK}s, no MCU)"
say ""
CFG_A=$OUT/no-hardware.cfg
# `[respond]` is what makes the load in stage B real: `RESPOND` is a plain
# command, so the flood is not rejected by the macro-recursion guard (which is
# what happened to a `[gcode_macro BENCH]` calling RESPOND without this
# section). `[virtual_sdcard]` is for stage F3.
printf '[printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n\n[respond]\n\n[virtual_sdcard]\npath: %s\n' "$OUT" > "$CFG_A"
PID=$(start_host "$CFG_A")
if ! wait_ready "$PID"; then
    fail "the host did not reach \`ready\` within 30 s (see \`$RAW/klippy.out\`)"
    kill -TERM "$PID" 2>/dev/null
    say ""
    echo "wrote $REPORT"
    exit 1
fi
T0=$(threads_of "$PID"); R0=$(rss_of "$PID")
LOG0_LINES=$(wc -l < "$RAW/klippy.log" 2>/dev/null || echo 0)
if ! D0=$(read_dispatch); then
    fail "\`system_stats\` answered without \`dispatch\`/\`dropped_logs\` — this binary predates the object, so the rest of stage A would be meaningless. Rebuild: \`cargo build --release\`"
    kill -TERM "$PID" 2>/dev/null
    say ""
    echo "wrote $REPORT"
    exit 1
fi
sleep "$SOAK"
T1=$(threads_of "$PID"); R1=$(rss_of "$PID")
LOG1_LINES=$(wc -l < "$RAW/klippy.log" 2>/dev/null || echo 0)
if ! D1=$(read_dispatch); then
    fail "the host stopped answering \`objects/query\` during the soak (see $RAW/klippy.log)"
    kill -TERM "$PID" 2>/dev/null
    say ""
    echo "wrote $REPORT"
    exit 1
fi

say "| measure | start | end |"
say "|---|---|---|"
say "| threads | $T0 | $T1 |"
say "| RSS (kB) | $R0 | $R1 |"
say "| \`dispatch.busy_avg_s\` / \`lateness_avg_s\` / \`over_threshold\` / \`dropped_logs\` | $D0 | $D1 |"
say ""
[ "$T0" = "$T1" ] && pass "thread count is flat over ${SOAK}s ($T0)" || fail "thread count moved ${T0} -> ${T1} over ${SOAK}s (a leak or a runtime restarted)"
GROW=$(( ${R1:-0} - ${R0:-0} ))
if [ "$GROW" -le 2048 ]; then
    pass "RSS grew ${GROW} kB over ${SOAK}s (≤ 2 MiB)"
else
    fail "RSS grew ${GROW} kB over ${SOAK}s (> 2 MiB: watch it over a longer soak)"
fi
DROPPED=$(echo "$D1" | awk '{print $4}')
DROP0=$(echo "$D0" | awk '{print $4}')
[ "${DROPPED:-x}" = "${DROP0:-y}" ] && pass "no log records dropped (\`dropped_logs\` = $DROPPED)" || fail "log records were dropped: $DROP0 -> $DROPPED (the writer cannot keep up: slow disk or too much logging)"
OVERT=$(echo "$D1" | awk '{print $3}')
[ "${OVERT:-x}" = "0" ] && pass "no dispatch round over the 50 ms threshold" || advisory "$OVERT dispatch round(s) over 50 ms (look for \`Reactor busy\` in $RAW/klippy.log)"
LAT=$(echo "$D1" | awk '{print $2}')
say "- info: mean callback lateness is ${LAT}s; 1-2 ms is the tokio timer wheel, not a stall"

# ---------------------------------------------------------------- B api latency
log "stage B: api latency (loaded: 8 gcode/script + 8 info per round)"
say ""
say "## B. API latency under load"
say ""
( for _ in $(seq 16); do
      for _ in $(seq 8); do
          timeout 10 "$BIN" api -a "tcp:127.0.0.1:$PORT" gcode/script '{"script":"RESPOND MSG=\"bench\""}' >/dev/null 2>&1 &
          timeout 10 "$BIN" api -a "tcp:127.0.0.1:$PORT" info '{}' >/dev/null 2>&1 &
      done
      wait
  done ) &
FLOOD=$!
# D wants a loaded window, so sample the threads while the flood runs.
"$PY" "$REPO/scripts/kx-sched.py" --pid "$PID" --interval 15 > "$RAW/sched.txt" 2>&1
kill -TERM $FLOOD 2>/dev/null; wait $FLOOD 2>/dev/null

RTT=$RAW/rtt.txt
: > "$RTT"
FAILS=0
for _ in $(seq 100); do
    S=$(date +%s%N)
    if ! timeout 10 "$BIN" api -a "tcp:127.0.0.1:$PORT" info '{}' >/dev/null 2>&1; then
        FAILS=$((FAILS + 1))
    fi
    E=$(date +%s%N)
    echo $(( (E - S) / 1000000 )) >> "$RTT"
done
STATS=$(sort -n "$RTT" | "$PY" -c 'import sys; v=[int(x) for x in sys.stdin]; v.sort(); print("%d %d %d %d %d" % (len(v), v[0], v[len(v)//2], v[min(len(v)-1, int(len(v)*.99))], v[-1]))')
read -r N MIN P50 P99 MAX <<<"$STATS"
say "| requests | min | p50 | p99 | max | failed |"
say "|---|---|---|---|---|---|"
say "| $N | ${MIN} ms | ${P50} ms | ${P99} ms | ${MAX} ms | $FAILS |"
say ""
[ "${FAILS:-1}" = "0" ] && pass "100 sequential requests, 0 failures/timeouts" || fail "$FAILS of 100 requests failed or timed out"
[ "${P50:-9999}" -le 20 ] && pass "p50 ${P50} ms (≤ 20 ms; includes client process spawn)" || advisory "p50 ${P50} ms is slow for one request (client spawn dominates on a card)"
[ "${P99:-9999}" -le 100 ] && pass "p99 ${P99} ms (≤ 100 ms)" || advisory "p99 ${P99} ms > 100 ms: something stalls the api runtime now and then"

# ---------------------------------------------------------------- C restart storm
log "stage C: $STORM x RESTART"
say ""
say "## C. restart storm ($STORM x RESTART)"
say ""
TC0=$(threads_of "$PID"); RC0=$(rss_of "$PID")
RESTARTS_BEFORE=$(grep -c 'Restarting the printer' "$RAW/klippy.log" 2>/dev/null | tr -d ' \n')
RESTARTS_BEFORE=${RESTARTS_BEFORE:-0}
OK=0; BAD=0
for _ in $(seq "$STORM"); do
    if timeout 20 "$BIN" api -a "tcp:127.0.0.1:$PORT" gcode/script '{"script":"RESTART"}' >/dev/null 2>&1; then
        OK=$((OK + 1))
    else
        BAD=$((BAD + 1))
    fi
    sleep 0.7
done
TC1=$(threads_of "$PID"); RC1=$(rss_of "$PID")
RESTARTS_AFTER=$(grep -c 'Restarting the printer' "$RAW/klippy.log" 2>/dev/null | tr -d ' \n')
RESTARTS_AFTER=${RESTARTS_AFTER:-0}
RESTARTS=$(( RESTARTS_AFTER - RESTARTS_BEFORE ))
say "| measure | before | after |"
say "|---|---|---|"
say "| RESTART requests accepted | - | $OK / $STORM |"
say "| restarts actually performed (from the log) | - | $RESTARTS |"
say "| threads | $TC0 | $TC1 |"
say "| RSS (kB) | $RC0 | $RC1 |"
say ""
[ "${BAD:-1}" = "0" ] && pass "every RESTART was accepted ($OK/$STORM)" || fail "$BAD of $STORM RESTARTs were refused or timed out"
if [ "${RESTARTS:-0}" -lt "${OK:-0}" ]; then
    advisory "only $RESTARTS of $OK accepted RESTARTs actually restarted: the rest landed inside a restart window and were coalesced (the accepted count measures the API, not the work)"
fi
[ "$TC0" = "$TC1" ] && pass "thread count is flat across $STORM restarts ($TC1)" || fail "thread count moved ${TC0} -> ${TC1} across $STORM restarts (leak per restart)"
RC_GROW=$(( ${RC1:-0} - ${RC0:-0} ))
if [ "$RC_GROW" -le 10240 ]; then
    pass "RSS grew ${RC_GROW} kB across $STORM restarts (≤ 10 MiB; growth that decelerates is the allocator, not a leak)"
else
    fail "RSS grew ${RC_GROW} kB across $STORM restarts (> 10 MiB)"
fi
STATE=""
for _ in $(seq 30); do
    STATE=$(query '{"objects":{"webhooks":["state"]}}' | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["status"]["webhooks"]["state"])' 2>/dev/null)
    [ "$STATE" = "ready" ] && break
    sleep 0.5
done
[ "$STATE" = "ready" ] && pass "host is \`ready\` again within 15 s of the last RESTART" || fail "host is \`${STATE:-unknown}\` 15 s after the last RESTART, not \`ready\`"

kill -TERM "$PID" 2>/dev/null
for _ in $(seq 20); do
    kill -0 "$PID" 2>/dev/null || break
    sleep 0.25
done
if kill -0 "$PID" 2>/dev/null; then
    fail "the host did not exit within 5 s of SIGTERM"
    kill -KILL "$PID" 2>/dev/null
else
    pass "clean exit on SIGTERM (log drained, no \`killed\` needed)"
fi

# ---------------------------------------------------------------- D jitter
say ""
say "## D. thread scheduling (loaded, ${SOAK}s window in B)"
say ""
if [ -s "$RAW/sched.txt" ]; then
    say '```'
    sed 's/^/  /' "$RAW/sched.txt" | head -14 >> "$REPORT"
    say '```'
    say ""
    WORST=$("$PY" - "$RAW/sched.txt" <<'EOF'
import re, sys
worst = 0.0
for line in open(sys.argv[1]):
    parts = line.split()
    if len(parts) < 6 or not parts[0].startswith("klippy-mcu"):
        continue
    try:
        worst = max(worst, float(parts[5]))
    except ValueError:
        pass
print(f"{worst:.2f}")
EOF
)
    say "- \`klippy-mcu\` worst mean run-queue wait: **${WORST} us**"
    awk -v w="$WORST" 'BEGIN { exit (w <= 100) ? 0 : 1 }' \
        && pass "machine workers wait ${WORST} us on the run queue (≤ 100 us)" \
        || advisory "machine worker waited ${WORST} us on average (a contended card shows this; see latency.md)"
else
    advisory "no sched sample captured (scripts/kx-sched.py failed; see $RAW/sched.txt)"
fi

# ---------------------------------------------------------------- F disk stalls
say ""
say "## F. disk stalls, and what they cost the host"
say ""
log "stage F: disk probe (${F_STALL:-20}s stall window)"
FJSON=$RAW/disk.json
if timeout 420 "$PY" "$REPO/scripts/disk-stall.py" --dir "${DISK_DIR:-$OUT}" \
        --size-mb "${DISK_MB:-64}" --stall-seconds "${F_STALL:-20}" > "$FJSON" 2> "$RAW/disk.err"; then
    FOUT=$("$PY" - "$FJSON" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
s = d.get("stall") or {}
print("| measure | value |")
print("|---|---|")
print(f"| sequential write / read | {d['seq_write_mbps']:.1f} / {d['seq_read_mbps']:.1f} MB/s |")
print(f"| 4 KiB random read (O_DIRECT) | {d['rand4k_iops']:.0f} IOPS, p50 {d['rand4k_p50_us']:.0f} us, p99 {d['rand4k_p99_us']:.0f} us, max {d['rand4k_max_us']:.0f} us |")
if 'rand4k_cached_p50_us' in d:
    print(f"| 4 KiB random read (page cache, for contrast) | {1e6/d['rand4k_cached_p50_us']:.0f} IOPS, p50 {d['rand4k_cached_p50_us']:.0f} us |")
if d.get('rand4k_direct_note'):
    print(f"| note | {d['rand4k_direct_note']} |")
if s:
    print(f"| under sustained write: reads p99/max | {s['read_p99_ms']:.2f} / {s['read_max_ms']:.2f} ms |")
    print(f"| under sustained write: writes(fsync) p99/max | {s['write_p99_ms']:.2f} / {s['write_max_ms']:.2f} ms |")
    print(f"| stalls over 50 / 200 / 1000 ms | reads {s['read_over_50ms']} / {s['read_over_200ms']} / {s['read_over_1000ms']}, writes {s['write_over_50ms']} / {s['write_over_200ms']} / {s['write_over_1000ms']} |")
    print(f"| operations measured | {s['reads']} reads, {s['writes']} writes in {s['seconds']:.0f} s |")
EOF
)
    say "$FOUT"
    say ""
    eval "$( "$PY" - "$FJSON" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
s = d.get("stall") or {}
print(f"F_IOPS={d['rand4k_iops']:.0f} F_P99_US={d['rand4k_p99_us']:.0f} F_RMAX={s.get('read_max_ms', 0):.1f} F_WMAX={s.get('write_max_ms', 0):.1f} F_OVER200={s.get('read_over_200ms', 0)} F_OVER1000={s.get('read_over_1000ms', 0)}")
EOF
)"
    if [ "${F_IOPS:-0}" -lt 500 ]; then
        advisory "4 KiB random read is ${F_IOPS} IOPS (< 500): this disk is degraded; the numbers below say what that costs"
    else
        pass "4 KiB random read is ${F_IOPS} IOPS (≥ 500)"
    fi
    if [ "${F_OVER1000:-0}" -gt 0 ]; then
        advisory "the controller stalled for over 1 s ${F_OVER1000} time(s) under write pressure — that is the shape that overflows a log queue"
    elif [ "${F_OVER200:-0}" -gt 0 ]; then
        advisory "${F_OVER200} read(s) took over 200 ms under write pressure (visible hitch, not fatal)"
    else
        pass "no read took over 200 ms while writing (max ${F_RMAX} ms; writes max ${F_WMAX} ms)"
    fi
else
    advisory "the disk probe failed (see $RAW/disk.err)"
fi

# F2: how long a stall the log queue can absorb.
#
# The host does not log per gcode command, so without a real board (and a real
# print) there is nothing to flood the logger with. What *is* knowable is the
# headroom: the queue holds LOG_QUEUE_CAPACITY records, the host produced
# `rate` records/s during the idle soak, so a disk stall of `capacity / rate`
# seconds can pass without losing a line. That is the number that decides
# whether this disk's stalls are dangerous.
LOG_QUEUE_CAPACITY=1024
RATE_LINES=$(awk -v a="${LOG0_LINES:-0}" -v b="${LOG1_LINES:-0}" -v t="$SOAK" 'BEGIN{ if (t>0) printf "%.1f", (b-a)/t; else print "0" }')
say "| log headroom | value |"
say "|---|---|"
say "| lines written during the idle soak | $((${LOG1_LINES:-0} - ${LOG0_LINES:-0})) in ${SOAK}s (${RATE_LINES}/s) |"
say "| queue capacity | ${LOG_QUEUE_CAPACITY} records |"
HEADROOM=$(awk -v c="$LOG_QUEUE_CAPACITY" -v r="$RATE_LINES" 'BEGIN{ if (r>0) printf "%.0f", c/r; else print "inf" }')
say "| stall this log rate tolerates | ${HEADROOM} s |"
say ""
if [ "$HEADROOM" = "inf" ]; then
    advisory "the host logged nothing during the soak, so the queue headroom is unbounded at this rate; a real print logs more (see the note in host-stability.md)"
else
    pass "at ${RATE_LINES} lines/s the queue tolerates a ${HEADROOM}s disk stall without dropping a line (measured stalls: ${F_RMAX:-?} ms)"
fi
say "- note: this host does not log per gcode command, so a *heavy* log rate is only reachable with a real board and a real print; at that point the same arithmetic applies to \`dropped_logs\`."

# F3: does a slow disk hitch the replay? (no MCU: the file is RESPOND lines)
log "stage F3: replay under this disk"
REPLAY=$OUT/replay.gcode
"$PY" - "$REPLAY" <<'EOF'
import sys
lines = 40000
with open(sys.argv[1], "w") as fh:
    for i in range(lines):
        fh.write(f'RESPOND MSG="line {i} of the replay probe"\n')
EOF
REPLAY_MB=$(( $(stat -c %s "$REPLAY") / 1048576 ))
PID=$(start_host "$CFG_A")
if wait_ready "$PID"; then
    "$BIN" api -a "tcp:127.0.0.1:$PORT" gcode/script '{"script":"M23 replay.gcode"}' >/dev/null 2>&1
    "$BIN" api -a "tcp:127.0.0.1:$PORT" gcode/script '{"script":"M24"}' >/dev/null 2>&1
    "$PY" - "$BIN" "$PORT" <<'EOF' > "$RAW/replay.txt" 2>&1
import json, subprocess, sys, time
binary, port = sys.argv[1], sys.argv[2]
def query():
    out = subprocess.run([binary, "api", "-a", f"tcp:127.0.0.1:{port}", "objects/query",
                          '{"objects":{"virtual_sdcard":["file_position","progress"],"system_stats":["dispatch"]}}'],
                         capture_output=True, text=True, timeout=20)
    return json.loads(out.stdout)["status"]
positions = []
worst = 0.0
started = time.time()
while time.time() - started < 45:
    try:
        st = query()
    except Exception:
        break
    pos = st["virtual_sdcard"]["file_position"]
    positions.append((time.time() - started, pos))
    if st["virtual_sdcard"]["progress"] >= 1.0:
        break
    time.sleep(0.1)
if len(positions) > 2:
    gaps = [(b[0] - a[0]) for a, b in zip(positions, positions[1:]) if b[1] == a[1]]
    worst = max(gaps) if gaps else 0.0
    moved = positions[-1][1] - positions[0][1]
    span = positions[-1][0] - positions[0][0]
    print(f"progress={positions[-1][1]} moved={moved} span={span:.1f}s max_stall={worst:.2f}s")
    print(f"bytes_per_second={moved / span if span else 0:.0f}")
else:
    print("progress=unavailable")
EOF
    RBPS=$(grep -o 'bytes_per_second=[0-9]*' "$RAW/replay.txt" | cut -d= -f2)
    RSTALL=$(grep -o 'max_stall=[0-9.]*' "$RAW/replay.txt" | cut -d= -f2)
    say "| replay (${REPLAY_MB} MiB file, no MCU) | value |"
    say "|---|---|"
    say "| read throughput | ${RBPS:-?} B/s |"
    say "| longest stall in file position | ${RSTALL:-?} s |"
    say ""
    RBPS_N=${RBPS:-0}
    if [ "$RBPS_N" -le 0 ]; then
        advisory "the replay never advanced (M23/M24 did not start it, or the file was empty), so F3 has no reading"
    elif [ -n "${RSTALL:-}" ] && [ "${RSTALL%%.*}" -le 1 ] 2>/dev/null; then
        pass "replay advanced at ${RBPS_N} B/s and never stalled for more than ~1 s (${RSTALL}s)"
    else
        advisory "replay position stalled for ${RSTALL:-?}s: the read is on the blocking pool so the machine keeps running, but the print would pause"
    fi
    kill -TERM "$PID" 2>/dev/null
    for _ in $(seq 20); do kill -0 "$PID" 2>/dev/null || break; sleep 0.25; done
else
    advisory "the replay host did not come up (skipped F3)"
    kill -TERM "$PID" 2>/dev/null
fi
rm -f "$REPLAY"

# ---------------------------------------------------------------- E real board
say ""
say "## E. real board (read-only)"
say ""
if [ -z "$CONFIG" ]; then
    say "- skipped: pass \`--config <printer.cfg>\` to include it"
else
    log "stage E: real board, read-only"
    PORT=${PORT_E:-7125}
    PID=$(start_host "$CONFIG")
    if ! wait_ready "$PID"; then
        fail "the real session did not reach \`ready\` (wrong \`-a\` port, or the MCU is not answering; see $RAW/klippy.out)"
    else
        pass "real session is \`ready\`"
        L1=$(query '{"objects":{"mcu":["last_stats","last_stats_time"]}}')
        sleep 10
        L2=$(query '{"objects":{"mcu":["last_stats","last_stats_time"]}}')
        echo "$L1" > "$RAW/last_stats_first.json"
        echo "$L2" > "$RAW/last_stats_second.json"
        RATE=$("$PY" - "$RAW/last_stats_first.json" "$RAW/last_stats_second.json" <<'EOF'
import json, sys
def load(p):
    st = json.load(open(p))["status"]["mcu"]
    return st["last_stats"], st["last_stats_time"]
try:
    a, at = load(sys.argv[1]); b, bt = load(sys.argv[2])
except Exception as exc:
    print(f"error {exc}"); raise SystemExit
dt = bt - at
if dt <= 0:
    print("error no time advanced"); raise SystemExit
read = (b["bytes_read"] - a["bytes_read"]) / dt
write = (b["bytes_write"] - a["bytes_write"]) / dt
print(f"{read:.0f} {write:.0f} {b.get('mcu_task_avg', 0)*1e6:.2f} {b.get('mcu_task_stddev', 0)*1e6:.2f} {'yes' if 'srtt' in b else 'no'}")
EOF
)
        read -r BREAD BWRITE AVG SD SRTT <<<"$RATE"
        say "| measure | value |"
        say "|---|---|"
        say "| read / write rate | ${BREAD} / ${BWRITE} B/s |"
        say "| \`mcu_task_avg\` / \`mcu_task_stddev\` | ${AVG} / ${SD} us |"
        say "| \`srtt\` present | ${SRTT} |"
        say ""
        [ "${BREAD:-0}" -gt 0 ] && pass "the host is exchanging bytes with the MCU (${BREAD} B/s read)" || fail "no bytes read from the MCU in 10 s (\`last_stats\` is not advancing)"
        [ "$SRTT" = "yes" ] && pass "\`srtt\`/rttvar/rto are published (round-trip estimation running)" || advisory "no \`srtt\` yet: no block has completed a round trip in this window"
        say "- note: this stage only reads. It does not home, heat or move anything."
        kill -TERM "$PID" 2>/dev/null
    fi
fi

# ---------------------------------------------------------------- summary
say ""
say "## summary"
say ""
if [ "$FAILED" -eq 0 ]; then
    say "**all hard checks passed** (${WARNED} advisory warning(s))"
else
    say "**${FAILED} hard check(s) failed**, ${WARNED} advisory warning(s)"
fi
say ""
say "Raw logs and samples: \`$RAW\`"
say ""

log "report: $REPORT"
if [ "$FAILED" -eq 0 ]; then
    exit 0
else
    exit 1
fi
