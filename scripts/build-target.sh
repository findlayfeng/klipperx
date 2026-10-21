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
GLIBC=""
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
        --glibc) GLIBC=$2; shift 2 ;;
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

# A glibc floor is a property of the sysroot used at link time, not of the
# architecture. With `--glibc X.Y` the build runs somewhere whose libc is that
# version: `cargo-zigbuild` (exact, no container) or a Debian container of the
# matching release (docker). Without it, the floor is whatever this machine's
# cross sysroot has, which is why every gnu product prints its own requirement.
debian_for_glibc() {
    case "$1" in
        2.36) echo bookworm ;;
        2.31) echo bullseye ;;
        2.28) echo buster ;;
        2.41) echo trixie ;;
        *) echo "" ;;
    esac
}

# Run the gnu build through a container of the Debian release that ships $GLIBC.
# Called only from the target loop; $1=$TRIPLE, $2=linker, $3=codename.
build_in_container() {
    local triple=$1 linker=$2 codename=$3 rel=${0#./}
    docker run --rm -v "$REPO:/w" -w /w "debian:$codename" bash -lc "
      set -e
      apt-get update -qq >/dev/null
      apt-get install -y -qq gcc-aarch64-linux-gnu gcc-x86-64-linux-gnu curl ca-certificates >/dev/null 2>&1 || apt-get install -y -qq gcc-aarch64-linux-gnu curl ca-certificates >/dev/null
      if ! command -v cargo >/dev/null 2>&1; then
          curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal >/dev/null
      fi
      . \\$HOME/.cargo/env
      rustup target add $triple >/dev/null
      $rel --target $triple
    "
}

# Copy the built binaries out of target/<triple> into out/cross/<triple>, with
# the size, hash and linkage each one has. Shared by every build path.
copy_products() {
    local TRIPLE=$1 DEST=$OUT/$1 found=0 bin size sha kind need
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
    [ "$found" -eq 0 ] && { echo "   FAILED: no binaries in target/$TRIPLE/$PROFILE"; FAILED=$((FAILED + 1)); }
}

FAILED=0
SKIPPED=0
SUMMARY=""

for TRIPLE in "${TARGETS[@]}"; do
    LINKER=$(linker_for "$TRIPLE")
    echo "== $TRIPLE"
    if [ -n "$GLIBC" ] && [ "${TRIPLE%-gnu}" != "$TRIPLE" ]; then
        if command -v cargo-zigbuild >/dev/null 2>&1 && command -v zig >/dev/null 2>&1; then
            echo "   path: cargo-zigbuild, glibc floor $GLIBC"
            if [ "$DRY" = "1" ]; then
                echo "   would run: cargo zigbuild --profile $PROFILE --target $TRIPLE.$GLIBC"
                continue
            fi
            if ! cargo zigbuild --profile "$PROFILE" --target "$TRIPLE.$GLIBC"; then
                echo "   FAILED: cargo zigbuild"
                FAILED=$((FAILED + 1)); continue
            fi
            copy_products
            continue
        fi
        codename=$(debian_for_glibc "$GLIBC")
        if [ -n "$codename" ] && command -v docker >/dev/null 2>&1; then
            echo "   path: debian:$codename container (glibc $GLIBC), linker $LINKER"
            if [ "$DRY" = "1" ]; then
                echo "   would run: docker run --rm -v \$PWD:/w -w /w debian:$codename … $0 --target $TRIPLE"
                continue
            fi
            if ! build_in_container "$TRIPLE" "$LINKER" "$codename"; then
                echo "   FAILED: container build"
                FAILED=$((FAILED + 1)); continue
            fi
            copy_products
            continue
        fi
        echo "   skipped: cannot pin glibc $GLIBC here — install zig+cargo-zigbuild (exact),"
        echo "            or docker (Debian container), or pass a matching --target toolchain"
        SKIPPED=$((SKIPPED + 1)); continue
    fi

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

    copy_products "$TRIPLE"
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
