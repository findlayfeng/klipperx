#!/bin/sh
#
# klipperx-usb-udev.sh — the udev rules `restart_method: rpi_usb` needs.
#
# A board is reset by switching the power of the USB port it sits on, so the
# **hub** behind each Klipper MCU is what has to be granted to the user that runs
# klipperx. This prints those rules; --install writes them to
# /etc/udev/rules.d and reloads udev.
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
    sed -n '3,17p' "$0" | sed 's/^# \{0,1\}//'
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

# The `<vendor> <product>` of the hub a USB device directory hangs off.
hub_line_of() {
    hub=$(dirname "$1")
    printf '%s %s\n' "$(cat "$hub/idVendor" 2>/dev/null || echo '????')" \
        "$(cat "$hub/idProduct" 2>/dev/null || echo '????')"
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
emit_rules() {
    echo "# klipperx: USB port power for \`restart_method: rpi_usb\`."
    if [ -n "$devices" ]; then
        echo "# Generated for: $devices"
    else
        echo "# Generated for the Klipper USB ids: $klipper_ids"
    fi
    echo "# See docs/klippy/user-manual/config.md."
    printf '%s\n' "$hubs" | while read -r vendor product; do
        # The loop is a subshell, but it only prints.
        [ -n "$vendor" ] && [ -n "$product" ] || continue
        echo
        echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vendor\", ATTR{idProduct}==\"$product\", TAG+=\"uaccess\""
        echo "SUBSYSTEM==\"usb\", DRIVER==\"hub|usb\", ATTR{idVendor}==\"$vendor\", ATTR{idProduct}==\"$product\", \\"
        echo "  RUN+=\"/bin/sh -c \\\"chmod -f 660 \$sys\$devpath/*port*/disable || true\\\"\""
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
