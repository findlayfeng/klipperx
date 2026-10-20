# klipperx firmware: one shared target configuration, two images.
#
# `make` builds katapult (the bootloader) and klipper (the application) from
# the pinned submodules under third_party/, using the shared `.config` resolved
# from the root `Kconfig` (see scripts/firmware-config.py for the derivation).
#
# Both images are built by the submodules' own build systems: their Makefiles
# are invoked with OUT and KCONFIG_CONFIG moved under this repository's
# out/firmware/, so the submodule checkouts stay read-only (nothing is written
# into third_party/).  That also makes the build work in a git worktree, where
# the submodules are empty: point KLIPPERX_KLIPPER_DIR / KLIPPERX_KATAPULT_DIR
# at the main checkout (the same variables the tests use) and each worktree
# keeps its own config and artifacts.
#
# There is **no committed seed** for `.config`: upstream ships none either, and
# a seed would have to name one particular board.  A machine without `.config`
# is told to run `make menuconfig`; the bench board's choices are written out in
# docs/klippy/user-manual/firmware-build.md.
#
# Variable names are deliberately not OUT/KCONFIG_CONFIG: those are the
# submodules' variables, and a value left in the environment must not redirect
# this build.

KLIPPER_DIR ?= $(if $(KLIPPERX_KLIPPER_DIR),$(KLIPPERX_KLIPPER_DIR),$(CURDIR)/third_party/klipper)
KATAPULT_DIR ?= $(if $(KLIPPERX_KATAPULT_DIR),$(KLIPPERX_KATAPULT_DIR),$(CURDIR)/third_party/katapult)

FIRMWARE_OUT ?= out/firmware
FIRMWARE_CONFIG ?= .config

KLIPPER_OUT := $(FIRMWARE_OUT)/klipper
KATAPULT_OUT := $(FIRMWARE_OUT)/katapult

PYTHON ?= $(if $(wildcard $(CURDIR)/venv/bin/python),$(CURDIR)/venv/bin/python,python3)

# The tools import kconfiglib from the checkouts' lib/ directories; do not leave
# __pycache__ behind in third_party/ (the Makefile's read-only claim).
export PYTHONDONTWRITEBYTECODE := 1

# One-line shell check used by the targets that read a checkout.
define require_tree
test -f $(1)/Makefile || { echo "firmware: $(1) has no build system (submodule not checked out)." >&2; echo "firmware: set KLIPPERX_KLIPPER_DIR / KLIPPERX_KATAPULT_DIR to a checked-out main tree." >&2; exit 1; }
endef

.PHONY: all firmware klipper katapult config olddefconfig menuconfig clean distclean help

all: firmware

# Both images; the artifacts stay in their build directories, one place each.
firmware: klipper katapult
	@for f in klipper.bin klipper.uf2; do \
		if [ -f $(KLIPPER_OUT)/$$f ]; then echo "firmware: $(abspath $(KLIPPER_OUT))/$$f"; fi; \
	done
	@echo "firmware: $(abspath $(KATAPULT_OUT))/katapult.bin"
	@if [ -f $(KATAPULT_OUT)/deployer.bin ]; then echo "firmware: $(abspath $(KATAPULT_OUT))/deployer.bin"; fi

# The application alone: only the Klipper half is derived, so this target works
# for any architecture the Klipper tree supports (the pair is checked by
# `make`/`make config`, which also derive the bootloader).
klipper: $(FIRMWARE_CONFIG)
	@$(call require_tree,$(KLIPPER_DIR))
	$(PYTHON) scripts/firmware-config.py --only klipper \
		--root $(CURDIR) --klipper $(KLIPPER_DIR) --katapult $(KATAPULT_DIR) \
		--config $(FIRMWARE_CONFIG) --out-dir $(FIRMWARE_OUT)
	$(MAKE) -C $(KLIPPER_DIR) OUT=$(abspath $(KLIPPER_OUT))/ \
		KCONFIG_CONFIG=$(abspath $(FIRMWARE_OUT)/klipper.config) PYTHON=$(PYTHON) all

katapult: $(FIRMWARE_CONFIG)
	@$(call require_tree,$(KLIPPER_DIR))
	@$(call require_tree,$(KATAPULT_DIR))
	$(PYTHON) scripts/firmware-config.py --only both \
		--root $(CURDIR) --klipper $(KLIPPER_DIR) --katapult $(KATAPULT_DIR) \
		--config $(FIRMWARE_CONFIG) --out-dir $(FIRMWARE_OUT)
	$(MAKE) -C $(KATAPULT_DIR) OUT=$(abspath $(KATAPULT_OUT))/ \
		KCONFIG_CONFIG=$(abspath $(FIRMWARE_OUT)/katapult.config) PYTHON=$(PYTHON) all

# Resolve the shared config and derive both build configs, checking that the
# bootloader can build the same target.  Runs on every build (kconfig parsing
# is cheap) but only rewrites a file when its content changed, so the
# submodules' incremental builds are not disturbed.
config: $(FIRMWARE_CONFIG)
	@$(call require_tree,$(KLIPPER_DIR))
	@$(call require_tree,$(KATAPULT_DIR))
	$(PYTHON) scripts/firmware-config.py --only both \
		--root $(CURDIR) --klipper $(KLIPPER_DIR) --katapult $(KATAPULT_DIR) \
		--config $(FIRMWARE_CONFIG) --out-dir $(FIRMWARE_OUT)

olddefconfig: config

# A missing `.config` is the normal state of a fresh clone: upstream ships no
# seed, and this repository does not either (a seed names one board).  Say so
# instead of writing a configuration nobody asked for.
$(FIRMWARE_CONFIG):
	@echo "firmware: no $@ in $(CURDIR)" >&2
	@echo "firmware: run 'make menuconfig' and pick your board and console;" >&2
	@echo "firmware: the bench board's choices are listed in" >&2
	@echo "firmware:   docs/klippy/user-manual/firmware-build.md" >&2
	@exit 1

# The merged menu: the Klipper tree plus kconfig/katapult.kconfig.  `source`
# lines resolve against $srctree, which is the Klipper checkout.
menuconfig: $(FIRMWARE_CONFIG)
	@$(call require_tree,$(KLIPPER_DIR))
	@srctree=$(KLIPPER_DIR) KCONFIG_CONFIG=$(abspath $(FIRMWARE_CONFIG)) \
		$(PYTHON) $(KLIPPER_DIR)/lib/kconfiglib/menuconfig.py $(CURDIR)/Kconfig

clean:
	@rm -rf $(FIRMWARE_OUT)

distclean: clean
	@rm -f $(FIRMWARE_CONFIG) $(FIRMWARE_CONFIG).old

help:
	@printf '%s\n' \
		'make               build both firmwares (katapult + klipper)' \
		'make klipper       build only the application (any klipper target)' \
		'make katapult      build only the bootloader (+ deployer when configured)' \
		'make menuconfig    edit the shared target and both option sets' \
		'make olddefconfig  resolve the shared config and derive both build configs' \
		'make clean         remove out/firmware' \
		'make distclean     also remove the working .config'
