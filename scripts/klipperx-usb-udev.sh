#!/bin/sh
#
# klipperx-usb-udev.sh — the udev rules `restart_method: rpi_usb` needs.
#
# A board is reset by switching the power of the USB port it sits on, so the
# **hub** behind each Klipper MCU is what has to be granted to the user that runs
# klipperx. This prints those rules; --install writes them to
# /etc/udev/rules.d and reloads udev.
#
# The hub granted is the **nearest** one — the hub the MCU is plugged into. With
# hubs stacked several deep, that is the innermost of them: a port's power switch
# lives on the hub above the port, and so does the control request.
#
# A rule cannot make a hub switch power it does not switch: `wHubCharacteristics`
# in the hub's own descriptor is what says whether a port can be powered off at
# all, and a hub that reports no power switching can only be *disconnected*. That
# is reported here, per hub, with the rules (and warned about on stderr), because
# `restart_method: rpi_usb` cannot reset a board on such a port — the board needs
# a real power cycle, or the MCU needs `restart_method: command`.
#
# Usage:
#   klipperx-usb-udev.sh [--install] [DEVICE ...]
#
# With no DEVICE, the Klipper USB ids (`1d50:614e` serial, `1d50:606f` CAN, or
# $KLIPPERX_USB_IDS) are searched for. Give serial device paths (`/dev/ttyACM0`,
# `/dev/serial/by-id/...`) when the firmware was built with other ids.
#
# See docs/klippy/user-manual/config.md for what the rules mean.
#
set -eu

rules_file=/etc/udev/rules.d/52-klipperx-usb.rules
# The ids Klipper's firmware uses by default (src/Kconfig USB_VENDOR_ID /
# USB_DEVICE_ID, and the gs_usb one the CAN build hardcodes).
klipper_ids=${KLIPPERX_USB_IDS:-"1d50:614e 1d50:606f"}
do_install=no

usage() {
    # The header comment is the help text; the `See docs` line ends it, so
    # editing above it does not shift a line range out of date.
    sed -n '3,/^# See docs/p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case $1 in
        -h|--help) usage 0 ;;
        -i|--install) do_install=yes; shift ;;
        --) shift; break ;;
        -*) echo "unknown option: $1" >&2; usage 1 ;;
        *) break ;;
    esac
done
devices="$*"

# `<name> <vendor> <product>` of the hub a USB device directory hangs off: the
# name finds the hub in sysfs (and tells a root hub, `usb<bus>`, from a hub behind
# one), the ids are what the rules match on.
hub_line_of() {
    hub=$(dirname "$1")
    printf '%s %s %s\n' "$(basename "$hub")" \
        "$(cat "$hub/idVendor" 2>/dev/null || echo '????')" \
        "$(cat "$hub/idProduct" 2>/dev/null || echo '????')"
}

# What a hub says it can do about the power of its ports, as comments for the
# rules file — and a warning when the answer means `rpi_usb` cannot work there.
#
# The answer is `wHubCharacteristics` in the hub's class descriptor, which sysfs
# does not carry: only the hub itself can be asked, with a control transfer that
# needs write access to its `/dev/bus/usb` node (root, or the rule this script
# installs). So this reads it with `lsusb -v` when it can, and says which command
# to run when it cannot — the check is worth doing, because a hub that reports
# `No power switching` makes both mechanisms pointless.
hub_power_note() {
    name=$1
    where=$2:$3
    case $name in
        usb*) where="$where, root hub" ;;
    esac

    bus=$(cat "/sys/bus/usb/devices/$name/busnum" 2>/dev/null || true)
    dev=$(cat "/sys/bus/usb/devices/$name/devnum" 2>/dev/null || true)
    mode=
    chars=
    if command -v lsusb >/dev/null 2>&1 && [ -n "$bus" ] && [ -n "$dev" ]; then
        dump=$(lsusb -v -s "$bus:$dev" 2>/dev/null || true)
        chars=$(printf '%s\n' "$dump" | sed -n 's/^ *wHubCharacteristic *\(0x[0-9a-fA-F]*\).*/\1/p' | head -1)
        mode=$(printf '%s\n' "$dump" | sed -n \
            's/^ *\(Per-port power switching\|Ganged power switching\|No power switching\).*/\1/p' | head -1)
    fi

    if [ -z "$mode" ]; then
        echo "# hub $name ($where): power switching unknown"
        if [ -n "$bus" ] && [ -n "$dev" ]; then
            echo "#   check with: sudo lsusb -v -s $bus:$dev | grep -i -A2 wHubCharacteristic"
        fi
        echo "#   A hub that reports 'No power switching' can only disconnect the board:"
        echo "#   use restart_method: command, or power the board off for real."
        return
    fi

    case $mode in
        'Per-port power switching')
            echo "# hub $name ($where): per-port power switching ${chars:+($chars) }— this is the useful case"
            ;;
        'Ganged power switching')
            echo "# hub $name ($where): ganged power switching ${chars:+($chars) }— every port"
            echo "#   of this hub goes down together, so anything else on it goes too"
            ;;
        *)
            echo "# hub $name ($where): $mode ${chars:+($chars) }— a port can be disconnected,"
            echo "#   but it cannot be powered off, so this hub cannot reset a board on it:"
            echo "#   use restart_method: command, or power the board off for real."
            echo "warning: hub $name ($where) reports no port power switching, so" \
                "'restart_method: rpi_usb' cannot reset a board on it" >&2
            ;;
    esac
}

if [ -n "$devices" ]; then
    # Resolve each device to its USB device, then to the hub above it. A device
    # that is not a USB tty is an error, not a silent skip.
    hubs=
    for dev in "$@"; do
        real=$(readlink -f "$dev" 2>/dev/null) || true
        [ -n "$real" ] || { echo "$dev: cannot resolve" >&2; exit 1; }

        name=$(basename "$real")
        node=$(readlink -f "/sys/class/tty/$name/device" 2>/dev/null) || true
        [ -n "$node" ] || { echo "$dev: is not a tty with a USB device behind it" >&2; exit 1; }

        # The nearest ancestor carrying `busnum` is the USB device.
        dir=$node
        while [ "$dir" != / ] && [ ! -f "$dir/busnum" ]; do dir=$(dirname "$dir"); done
        [ -f "$dir/busnum" ] || { echo "$dev: is not a USB device" >&2; exit 1; }

        hubs="$hubs
$(hub_line_of "$dir")"
    done
    hubs=$(printf '%s\n' "$hubs" | sed '/^$/d' | sort -u)
else
    # No path given: find the devices by the ids the firmware reports.
    hubs=$(
        for link in /sys/bus/usb/devices/*; do
            dev=$(readlink -f "$link" 2>/dev/null) || continue
            [ -f "$dev/idVendor" ] && [ -f "$dev/idProduct" ] || continue
            pair=$(cat "$dev/idVendor"):$(cat "$dev/idProduct")
            case " $klipper_ids " in
                *" $pair "*) hub_line_of "$dev" ;;
            esac
        done | sort -u
    )
    if [ -z "$hubs" ]; then
        echo "no Klipper USB device found ($klipper_ids);" >&2
        echo "give serial device paths instead, or set KLIPPERX_USB_IDS" >&2
        exit 1
    fi
fi

# The rules, one pair per hub: the id-matched rule grants the `/dev/bus/usb`
# node (the hub control request), and the RUN+= rule makes the sysfs port switch
# writable — `MODE=` applies to `/dev` nodes, not to sysfs.
#
# The glob is one level *below* the hub, because `$devpath` is the hub's own
# device directory and its ports hang off the interface directory under it
# (`<hub>:<cfg>.<if>/<hub>-port<N>/disable`). Matching `$devpath/*port*/disable`
# instead would reach only the hub's `port` symlink — the port of the *next* hub
# up that feeds this one — and would match nothing at all on a root hub.
emit_rules() {
    echo "# klipperx: USB port power for \`restart_method: rpi_usb\`."
    if [ -n "$devices" ]; then
        echo "# Generated for: $devices"
    else
        echo "# Generated for the Klipper USB ids: $klipper_ids"
    fi
    echo "# See docs/klippy/user-manual/config.md."
    # One pair of rules per hub, but the rules name a hub by its ids: two hubs of
    # one model are the same rule, so `sort -u -k 2,3` keeps one line per id pair
    # (the report above it is for the hub that line came from).
    printf '%s\n' "$hubs" | sort -u -k 2,3 | while read -r name vendor product; do
        # The loop is a subshell, but it only prints.
        [ -n "$vendor" ] && [ -n "$product" ] || continue
        echo
        hub_power_note "$name" "$vendor" "$product"
        echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vendor\", ATTR{idProduct}==\"$product\", TAG+=\"uaccess\""
        echo "SUBSYSTEM==\"usb\", DRIVER==\"hub|usb\", ATTR{idVendor}==\"$vendor\", ATTR{idProduct}==\"$product\", \\"
        echo "  RUN+=\"/bin/sh -c \\\"chmod -f 660 \$sys\$devpath/*/*port*/disable || true\\\"\""
    done
}

rules=$(emit_rules)

if [ "$do_install" = no ]; then
    printf '%s\n' "$rules"
    echo "# install with: $0 --install${devices:+ $devices}" >&2
    exit 0
fi

printf '%s\n' "$rules" | sudo sh -c "cat > '$rules_file'"
echo "wrote $rules_file" >&2
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
echo "rules reloaded" >&2
