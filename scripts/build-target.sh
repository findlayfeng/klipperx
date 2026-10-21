#!/usr/bin/env bash
# build-target.sh - build the host binaries for another architecture.
#
# `cargo build --release` on a card-sized machine (RK3328/RK3566: 4x Cortex-A53)
# takes 12-20 minutes, of which one crate is 9 minutes of single-threaded codegen
# (`cargo build --release --timings` shows it). Building here and copying the
# binary over takes seconds, so this script is the supported way to do it.
#
#   scripts/build-target.sh                    # aarch64 + x86_64, glibc
#   scripts/build-target.sh --musl             # the same two, static (musl)
#   scripts/build-target.sh --target aarch64-unknown-linux-musl
#   scripts/build-target.sh --all --dry-run    # show what it would run
#
# Output goes to out/cross/<triple>/ (binaries + sha256 + `file` line), not into
# target/, so a cross build never disturbs the native one.
#
# # Which libc? (this matters more than the architecture)
#
# A `*-gnu` build links the *build machine's* glibc and refuses to start on an
# older one ("GLIBC_2.38 not found"). Building aarch64-gnu on a rolling-release
# x86 box and running it on Debian 12 (glibc 2.36) is exactly that trap.
# `*-musl` is statically linked, so it runs anywhere — that is the recommended
# choice for a card machine, with one caveat: a static musl binary cannot
# `dlopen`, so `host_library:` ([mcu] with a klipper host library) will not work
# there. Everything else (USB, serial, CAN, TUI, Mainsail/Moonraker helpers,
# config-URL include) is plain syscalls and works.
set -u

REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$REPO/out/cross}
JOBS=${JOBS:-}
PROFILE=release
DRY=0
TARGETS=()

usage() { sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --target) TARGETS+=("$2"); shift 2 ;;
        --all) TARGETS=(x86_64-unknown-linux-gnu x86_64-unknown-linux-musl \
                        aarch64-unknown-linux-gnu aarch64-unknown-linux-musl); shift ;;
        --musl) MUSL=1; shift ;;
        --gnu) MUSL=0; shift ;;
        --jobs) JOBS=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --profile) PROFILE=$2; shift 2 ;;
        --dry-run) DRY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

# Default: the two architectures this project is actually deployed on.
if [ ${#TARGETS[@]} -eq 0 ]; then
    if [ "${MUSL:-0}" = "1" ]; then
        TARGETS=(x86_64-unknown-linux-musl aarch64-unknown-linux-musl)
    else
        TARGETS=(x86_64-unknown-linux-gnu aarch64-unknown-linux-gnu)
    fi
fi

# triple -> (linker, C compiler). cc-rs (used by `ring` for its C/asm) looks at
# CC_<triple with underscores>; cargo looks at CARGO_TARGET_<TRIPLE>_LINKER.
linker_for() {
    case "$1" in
        x86_64-unknown-linux-gnu)  echo cc ;;
        aarch64-unknown-linux-gnu) echo aarch64-linux-gnu-gcc ;;
        x86_64-unknown-linux-musl) echo x86_64-linux-musl-gcc ;;
        aarch64-unknown-linux-musl) echo aarch64-linux-musl-gcc ;;
        *) echo "" ;;
    esac
}

install_hint() {
    case "$1" in
        aarch64-unknown-linux-gnu)
            echo "  rustup target add aarch64-unknown-linux-gnu"
            echo "  sudo apt install gcc-aarch64-linux-gnu    # Debian/Ubuntu" ;;
        aarch64-unknown-linux-musl)
            echo "  rustup target add aarch64-unknown-linux-musl"
            echo "  # Debian has no aarch64-musl gcc; use a musl cross toolchain:"
            echo "  #   https://musl.cc/aarch64-linux-musl-cross.tgz  (then put bin/ on PATH)"
            echo "  # or: cargo install cross && cross build --release --target $1" ;;
        x86_64-unknown-linux-musl)
            echo "  rustup target add x86_64-unknown-linux-musl"
            echo "  sudo apt install musl-tools               # provides x86_64-linux-musl-gcc" ;;
        *)
            echo "  rustup target add $1"; echo "  # and a C compiler for it (see rustc --print target-list)" ;;
    esac
}

FAILED=0
SKIPPED=0
SUMMARY=""

for TRIPLE in "${TARGETS[@]}"; do
    LINKER=$(linker_for "$TRIPLE")
    echo "== $TRIPLE"
    if [ -z "$LINKER" ]; then
        echo "   skipped: no linker mapping in this script for that triple"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi
    if ! rustup target list --installed 2>/dev/null | grep -qx "$TRIPLE"; then
        echo "   skipped: rust target not installed"
        install_hint "$TRIPLE" | sed 's/^/     /'
        SKIPPED=$((SKIPPED + 1))
        continue
    fi
    if ! command -v "$LINKER" >/dev/null 2>&1; then
        echo "   skipped: linker '$LINKER' not found"
        install_hint "$TRIPLE" | sed 's/^/     /'
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    UPPER=$(echo "$TRIPLE" | tr 'a-z-' 'A-Z_')
    UNDER=$(echo "$TRIPLE" | tr '-' '_')
    export "CARGO_TARGET_${UPPER}_LINKER=$LINKER"
    export "CC_${UNDER}=$LINKER"
    # cc-rs also wants an archiver for some C builds; only set it when present.
    for candidate in "${LINKER%gcc}ar" ar; do
        if command -v "$candidate" >/dev/null 2>&1 && [ "$candidate" != "ar" ]; then
            export "AR_${UNDER}=$candidate"
            break
        fi
    done
    [ -n "$JOBS" ] && export CARGO_BUILD_JOBS="$JOBS"

    echo "   linker: $LINKER  (CARGO_TARGET_${UPPER}_LINKER, CC_${UNDER})"
    if [ "$DRY" = "1" ]; then
        echo "   would run: cargo build --profile $PROFILE --target $TRIPLE"
        continue
    fi

    if ! cargo build --profile "$PROFILE" --target "$TRIPLE"; then
        echo "   FAILED: cargo build"
        FAILED=$((FAILED + 1))
        continue
    fi

    DEST=$OUT/$TRIPLE
    mkdir -p "$DEST"
    found=0
    for bin in klipperx klippy klippy-client; do
        src=$REPO/target/$TRIPLE/$PROFILE/$bin
        [ -f "$src" ] || continue
        cp "$src" "$DEST/$bin"
        size=$(stat -c %s "$DEST/$bin")
        sha=$(sha256sum "$DEST/$bin" | cut -d' ' -f1)
        if ldd "$DEST/$bin" 2>&1 | grep -q 'not a dynamic executable\|statically linked'; then
            kind="static ($(file -b "$DEST/$bin" | cut -d, -f2 | sed 's/^ *//'))"
        else
            kind="dynamic: $(ldd "$DEST/$bin" 2>/dev/null | awk '{print $1}' | paste -sd' ' - | cut -c1-60)"
        fi
        printf '   %-14s %8.1f MB  %s\n' "$bin" "$(awk -v s="$size" 'BEGIN{printf "%.1f", s/1048576}')" "$kind"
        printf '   %-14s sha256 %s\n' "" "$sha"
        SUMMARY="$SUMMARY$TRIPLE $bin $(awk -v s="$size" 'BEGIN{printf "%.1f", s/1048576}')MB $sha\n"
        found=$((found + 1))
        # A gnu binary carries a glibc floor; say which, so it is compared with
        # the target's `ldd --version` before someone copies it over.
        case "$TRIPLE" in
            *-gnu)
                need=$(readelf --version-info "$DEST/$bin" 2>/dev/null \
                       | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1)
                [ -n "$need" ] && echo "   requires at least $need (check on the target: ldd --version)"
                ;;
            *-musl)
                echo "   static musl: runs on any glibc; cannot dlopen, so host_library: is unavailable"
                ;;
        esac
    done
    [ "$found" -eq 0 ] && { echo "   FAILED: no binaries in target/$TRIPLE/$PROFILE"; FAILED=$((FAILED + 1)); }
done

echo
echo "== summary"
if [ -n "$SUMMARY" ]; then
    printf '%b' "$SUMMARY" | sed 's/^/   /'
else
    echo "   (nothing built)"
fi
echo "   output: $OUT"
[ "$SKIPPED" -gt 0 ] && echo "   $SKIPPED target(s) skipped for missing prerequisites (see hints above)"
[ "$FAILED" -gt 0 ] && echo "   $FAILED target(s) failed" && exit 1
exit 0
