#!/usr/bin/env python3
"""Derive the two firmware build configs from the one shared target config.

The root ``Kconfig`` is the merged configuration UI.  Its target menu is the
Klipper tree itself (``third_party/klipper/src/Kconfig``): katapult's Kconfig
defines the same target symbols with the same values, so the architecture, the
processor model, the clock reference, the communication interface and the
bootloader offset are asked once and both firmwares get the answer.

This script turns that one resolved config into the two config files the two
in-tree build systems consume:

* ``<out-dir>/klipper.config`` -- the shared config re-resolved through
  Klipper's own Kconfig.  The tree is the same one the UI sources, so nothing
  is added and the file parses warning-free in klipper's ``genconfig.py``.
* ``<out-dir>/katapult.config`` -- the shared values that katapult's own
  Kconfig offers as user-selectable options (same names, same meanings),
  re-resolved through katapult's Kconfig so that katapult derives the rest
  (MCU, clock, flash geometry, HAVE_* capability selects) exactly as an
  in-tree katapult build would.

What is not asked twice:

* the application slot: ``FLASH_APPLICATION_ADDRESS`` (klipper) is katapult's
  ``LAUNCH_APP_ADDRESS``; the bootloader offset choice also selects katapult's
  older ``STM32_APP_START_*`` mirror of that choice.
* the USB device id: per firmware on purpose.  katapult keeps the id flashing
  tools look for (0x6177), klipper keeps its own (0x614e).  The vendor id is
  shared.  ``LOW_LEVEL_OPTIONS`` is also not forwarded: katapult's has no
  prompt and defaults to on.
* the deployer: katapult's ``BUILD_DEPLOYER`` is derived from the bootloader
  offset by its own Kconfig, so a "no bootloader" build does not get one.

The script also validates that the bootloader really can build the configured
target (architecture, MCU, clock, transports, application slot) and fails with
an explanation instead of silently building a different board.
"""

import argparse
import os
import sys

# Symbols the trees define under the same name but with a per-firmware value.
PER_FIRMWARE_SYMBOLS = {
    "LOW_LEVEL_OPTIONS": (
        "katapult's has no prompt and is always on; the shared flag only "
        "gates klipper's option visibility"
    ),
    "USB_DEVICE_ID": (
        "katapult keeps 0x6177 (the id flashing tools look for), klipper "
        "keeps 0x614e"
    ),
}

# klipper's bootloader offset choice -> katapult's mirror of the same choice.
APP_START_MIRROR = ("STM32_FLASH_START_", "STM32_APP_START_")

TRANSPORT_SYMBOLS = ("SERIAL", "USBSERIAL", "CANSERIAL", "USBCANBUS")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--root", required=True, help="repository root (the directory holding Kconfig)"
    )
    parser.add_argument(
        "--klipper", required=True, help="klipper checkout (KLIPPERX_KLIPPER_DIR)"
    )
    parser.add_argument(
        "--katapult", required=True, help="katapult checkout (KLIPPERX_KATAPULT_DIR)"
    )
    parser.add_argument(
        "--config",
        required=True,
        help="shared working config (read, then rewritten in canonical form)",
    )
    parser.add_argument(
        "--out-dir", required=True, help="directory for the derived per-firmware configs"
    )
    parser.add_argument(
        "--only",
        choices=("both", "klipper"),
        default="both",
        help=(
            "'klipper' derives only the application config and skips the "
            "bootloader derivation and its target check (default: both)"
        ),
    )
    args = parser.parse_args()
    # Absolute from here on: load_tree() chdirs into a tree before reading the
    # Kconfig file, so a relative path would be resolved against the wrong cwd.
    for name in ("root", "klipper", "katapult", "config", "out_dir"):
        setattr(args, name, os.path.abspath(getattr(args, name)))
    return args


def import_kconfiglib(klipper_dir):
    """Use the kconfiglib the trees themselves ship, not an installed one."""
    # Importing from the checkout must not leave __pycache__ in it: the build
    # treats the submodule trees as read-only.
    sys.dont_write_bytecode = True
    sys.path.insert(0, os.path.join(klipper_dir, "lib", "kconfiglib"))
    import kconfiglib

    return kconfiglib


def load_tree(kconfiglib, kconfig_path, srctree):
    """Parse one Kconfig tree the way its own in-tree tools do.

    ``source`` is resolved against $srctree, and ``$(shell,...)`` runs with the
    tree's own working directory (katapult's mainmenu runs a version script
    there).
    """
    previous_srctree = os.environ.get("srctree")
    previous_cwd = os.getcwd()
    os.environ["srctree"] = srctree
    os.chdir(srctree)
    try:
        return kconfiglib.Kconfig(kconfig_path, warn=True, warn_to_stderr=False)
    finally:
        os.chdir(previous_cwd)
        if previous_srctree is None:
            del os.environ["srctree"]
        else:
            os.environ["srctree"] = previous_srctree


def write_config_if_changed(kconf, path):
    """Write the canonical config, but leave the file (and its mtime) alone when
    the content did not change: the build systems rebuild on config mtime."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    # Unique name: `make -j` can run the two `--only` derivations at once, and
    # both rewrite the shared .config.
    tmp = f"{path}.tmp.{os.getpid()}"
    kconf.write_config(tmp, save_old=False)
    with open(tmp, encoding="utf-8") as handle:
        new = handle.read()
    try:
        with open(path, encoding="utf-8") as handle:
            old = handle.read()
    except FileNotFoundError:
        old = None
    if new == old:
        os.remove(tmp)
    else:
        os.replace(tmp, path)
    return new != old


def prompted(symbol):
    """True for symbols a user can select (choice items included)."""
    return any(node.prompt is not None for node in symbol.nodes)


def symbol(kconf, name):
    sym = kconf.syms.get(name)
    if sym is None or not sym.nodes:
        return None
    return sym


def value(kconf, name):
    sym = symbol(kconf, name)
    return sym.str_value if sym is not None else ""


def tri_value(kconf, name):
    sym = symbol(kconf, name)
    return sym.tri_value if sym is not None else 0


def forward_shared_values(kconfiglib, shared, boot):
    """Copy every shared, user-selectable value onto katapult's own tree.

    The rule is deliberately "the bootloader's Kconfig offers it too": what
    katapult does not offer as an option is katapult's to derive, and what it
    offers is what the user answered in the merged UI.
    """
    forwarded = []
    not_offered = []
    for sym in shared.unique_defined_syms:
        if sym.name in PER_FIRMWARE_SYMBOLS or not prompted(sym):
            continue
        target = symbol(boot, sym.name)
        if target is None or not prompted(target):
            continue
        wanted = sym.str_value
        if sym.type in (kconfiglib.BOOL, kconfiglib.TRISTATE):
            if wanted not in ("y", "m"):
                continue
        elif wanted == "":
            continue
        target.set_value(wanted)
        if target.str_value == wanted:
            forwarded.append(sym.name)
        else:
            not_offered.append((sym.name, wanted, target.str_value))
    return forwarded, not_offered


def parse_int(text):
    """Parse a numeric Kconfig value (hex or decimal); None when unset."""
    try:
        return int(text, 0)
    except ValueError:
        return None


def selected_stm32_offset(shared):
    """The chosen STM32 bootloader offset, e.g. "2000" (the choice item)."""
    for sym in shared.unique_defined_syms:
        if sym.name.startswith("STM32_FLASH_START_") and sym.str_value == "y":
            return sym.name[len("STM32_FLASH_START_") :]
    return None


def boot_app_offsets(boot):
    """The application offsets katapult's own Kconfig offers for this target."""
    return sorted(
        sym.name[len("STM32_APP_START_") :]
        for sym in boot.unique_defined_syms
        if sym.name.startswith("STM32_APP_START_")
        and prompted(sym)
        and sym.visibility
    )


def derive_bootloader_target(shared, boot):
    """Values katapult cannot derive from the shared symbols by name alone.

    katapult's application slot is a second choice (STM32_APP_START_*) older
    than the shared FLASH_START_* one, and LAUNCH_APP_ADDRESS is a promptless
    symbol that only follows that choice - a config-file value would be
    ignored by its own genconfig.  So the shared choice is mirrored onto the
    item of the same name, and validate() reports the offsets katapult cannot
    express at all.
    """
    notes = []
    shared_prefix, boot_prefix = APP_START_MIRROR
    for sym in shared.unique_defined_syms:
        if not sym.name.startswith(shared_prefix) or sym.str_value != "y":
            continue
        name = boot_prefix + sym.name[len(shared_prefix) :]
        target = symbol(boot, name)
        if target is None or not prompted(target) or not target.visibility:
            continue
        target.set_value("y")
        if target.str_value == "y":
            notes.append(f"{name}=y (bootloader offset)")
            launch = symbol(boot, "LAUNCH_APP_ADDRESS")
            if launch is not None:
                notes.append(f"LAUNCH_APP_ADDRESS={launch.str_value}")
    return notes


def selected_choice_items(shared):
    """The items the shared target selected in its choices (arch, model, clock,
    communication interface, bootloader offset).  katapult must select the same
    item: a missing one falls back to another pinout/processor silently."""
    return {
        sym.name
        for sym in shared.unique_defined_syms
        if sym.choice is not None and sym.str_value == "y"
    }


def selected_mach_symbols(shared):
    """The architecture/processor/variant symbols the shared target selected.
    Variants (MACH_STM32F070x6, MACH_N32G45x) are not choice items, and a
    bootloader built for the base model is not the same silicon."""
    return {
        sym.name
        for sym in shared.unique_defined_syms
        if sym.name.startswith("MACH_") and sym.str_value == "y"
    }


def validate(shared, boot):
    """Refuse to build a katapult that is not the configured target."""
    problems = []

    # FLASH_SIZE decides what the bootloader will write (and where the slot
    # check stops), so a disagreement means one tree's chip geometry is stale;
    # RAM_SIZE/CLOCK_FREQ only affect each image's own link/runtime and stay
    # per tree (noted below).
    for name in ("BOARD_DIRECTORY", "MCU", "FLASH_SIZE"):
        wanted, got = value(shared, name), value(boot, name)
        if wanted != got:
            problems.append(f"{name}: shared target has {wanted!r}, katapult resolved {got!r}")

    # The same processor, the same variant, the same communication pins.  A
    # symbol katapult's Kconfig does not have falls back to its first visible
    # item; on some pairs that lands on the same MCU strings (N32G45x ->
    # STM32F103) or the same transport kind (USART5 -> USART1), so the
    # comparisons above would not notice.
    for name in sorted(selected_choice_items(shared) | selected_mach_symbols(shared)):
        if tri_value(boot, name) != 2:
            problems.append(
                f"{name}: selected by the shared target but not by katapult's "
                f"Kconfig for this target (it would pick a different item)"
            )

    app = value(shared, "FLASH_APPLICATION_ADDRESS")
    launch = value(boot, "LAUNCH_APP_ADDRESS")
    slot_problem = False

    # katapult's application slot is its own choice on STM32; report the
    # offsets it cannot express before the generic address comparison below
    # (which would only say "0x8002000 vs 0x8005000").
    if tri_value(shared, "MACH_STM32"):
        offset = selected_stm32_offset(shared)
        if offset == "0000":
            problems.append(
                "the target has no bootloader slot ('No bootloader'), but "
                "katapult is a bootloader: pick a 4/8/16/32/128 KiB offset"
            )
            slot_problem = True
        elif offset is not None:
            target = symbol(boot, "STM32_APP_START_" + offset)
            if target is None or not target.visibility:
                offered = ", ".join(boot_app_offsets(boot))
                problems.append(
                    f"bootloader offset '{offset}' is not offered by katapult's "
                    f"Kconfig for this processor (application start offsets: {offered})"
                )
                slot_problem = True

    if not slot_problem and app and launch and parse_int(app) != parse_int(launch):
        problems.append(
            f"application slot: shared target starts at {app}, "
            f"katapult would launch {launch}"
        )

    for name in TRANSPORT_SYMBOLS:
        wanted, got = tri_value(shared, name), tri_value(boot, name)
        if wanted != got:
            problems.append(
                f"transport {name}: shared target has {bool(wanted)}, "
                f"katapult resolved {bool(got)}"
            )
    return problems


def fail(problems):
    # The derivation report so far goes to stdout; flush it first so the error
    # below reads in order even when the two streams are captured separately.
    sys.stdout.flush()
    sys.stderr.write("firmware: katapult cannot build the configured target:\n")
    for problem in problems:
        sys.stderr.write(f"  - {problem}\n")
    sys.stderr.write(
        "firmware: katapult's Kconfig supports the architectures, processor models\n"
        "firmware: and options it lists (STM32, LPC176x, RP2040/RP235x), with its\n"
        "firmware: own flash/RAM geometry.  Pick a target both trees have (the\n"
        "firmware: shared target menu is the Klipper tree's own).\n"
    )
    sys.exit(1)


def main():
    args = parse_args()
    kconfiglib = import_kconfiglib(args.klipper)

    if not os.path.exists(args.config):
        sys.exit(f"firmware: {args.config} does not exist; run 'make defconfig' first")

    # The merged UI: Klipper's tree for the shared target, plus the
    # bootloader-only menu from kconfig/katapult.kconfig.
    shared = load_tree(kconfiglib, os.path.join(args.root, "Kconfig"), args.klipper)
    shared.load_config(args.config)
    # Canonical form of the working config (what a menuconfig save produces):
    # the derivations below read the resolved values from it.
    write_config_if_changed(shared, args.config)

    klipper = load_tree(
        kconfiglib, os.path.join(args.klipper, "src", "Kconfig"), args.klipper
    )
    klipper.load_config(args.config)
    klipper_path = os.path.join(args.out_dir, "klipper.config")
    write_config_if_changed(klipper, klipper_path)

    print(
        "firmware: target {board} / {mcu} @ {clock} Hz, application at {app}".format(
            board=value(shared, "BOARD_DIRECTORY"),
            mcu=value(shared, "MCU"),
            clock=value(shared, "CLOCK_FREQ"),
            app=value(shared, "FLASH_APPLICATION_ADDRESS") or "n/a",
        )
    )
    print(f"firmware: {klipper_path}")

    if args.only == "klipper":
        return

    boot = load_tree(
        kconfiglib, os.path.join(args.katapult, "src", "Kconfig"), args.katapult
    )
    forwarded, not_offered = forward_shared_values(kconfiglib, shared, boot)
    derived = derive_bootloader_target(shared, boot)

    problems = validate(shared, boot)
    if problems:
        fail(problems)

    # A different clock or RAM size is fine: each image links and runs on its
    # own (katapult's defaults lag klipper's for a couple of models, and its
    # RAM reservation includes the ISP buffer), and the two never run at once.
    for name in ("CLOCK_FREQ", "RAM_SIZE"):
        shared_value = value(shared, name)
        boot_value = value(boot, name)
        if shared_value != boot_value:
            print(
                f"firmware: note: {name} {boot_value} in katapult vs "
                f"{shared_value} in klipper (each tree's own default for this MCU)"
            )

    boot_path = os.path.join(args.out_dir, "katapult.config")
    write_config_if_changed(boot, boot_path)

    print(f"firmware: {boot_path}  ({len(forwarded)} shared options forwarded)")
    for name, wanted, got in not_offered:
        print(
            f"firmware: katapult does not offer {name} for this target "
            f"({wanted} -> {got}); keeping katapult's value"
        )
    for note in derived:
        print(f"firmware: katapult derived {note}")
    for name, reason in PER_FIRMWARE_SYMBOLS.items():
        print(f"firmware: not merged, {name}: {reason}")


if __name__ == "__main__":
    main()
