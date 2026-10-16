#!/bin/sh
# PROPOSAL — run every test that loads the real klipper host library, one test
# per process. Not wired into the repo's gate: that stays `cargo test
# --workspace` (see AGENTS.md), which runs one process per test *binary*.
#
# Why it would help: the host library is one library per **process**. The
# registry is keyed by path and a load refuses a second, different live path
# outright (`refuse_a_second_library`, src/core/klippy/interface/devices/host.rs),
# because klipper stops its main loop through a process-wide SIGUSR1 handler that
# each library installs over the last one. Under `cargo test` all of these tests
# share one process, so they meet each other in the registry and are serialized
# behind `HOST_LIBRARY_TEST_LOCK`; `cargo nextest` gives each test a process of
# its own, and that lock is then uncontended by construction.
#
# Every test's bridge port now names its own tty (`[uart_device] uart_path:`, or
# the `(slot, tty)` table a host-library bridge is opened with), so no build-wide
# file is shared any more and there is nothing left to run one at a time.
#
# Printed here for a discussion of the gate, not as a second gate:
#   scripts/nextest-host-library.sh
# A git worktree has no submodules; point it at the main checkout first:
#   KLIPPERX_KLIPPER_DIR=<main checkout>/third_party/klipper scripts/nextest-host-library.sh
set -eu

repo=$(cd "$(dirname "$0")/.." && pwd)
cd "$repo"

# The modules whose tests load the real library (the interface one is a single
# test of its own). Over-inclusive on purpose: a test here that does not load the
# library costs one process, not correctness.
library='test(host_tests) | test(bridge_attach::tests) | test(restart::tests) | test(bridge_session) | test(scan::tests) | test(test_a_non_mux_host_library_has_no_tunnel_command)'

cargo nextest run -p klipperx --lib -E "$library"
