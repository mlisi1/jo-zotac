#!/usr/bin/env bash
# Checks that the host settings of install_host_rules.sh are installed AND in effect right now
# (udev rules, usbfs limit, CPU idle service), plus how the RealSense cameras and the Xsens are
# connected. Read-only, no sudo needed. Run on the host (not in the container):
#   bash host/check_host_rules.sh
# Exit code: the number of failed checks (0 = all good).
DIR="$(cd "$(dirname "$0")" && pwd)"
FAILS=0
if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'; else G= R= Y= B= N=; fi
ok()   { echo "  ${G}OK${N}    $*"; }
fail() { echo "  ${R}FAIL${N}  $*"; FAILS=$((FAILS + 1)); }
info() { echo "  ${Y}--${N}    $*"; }

# Serial numbers of the cameras, as in jo_bringup.launch.py
declare -A CAM=([318122300396]=front [327522300913]=back)

# Walks up from a sysfs path to the USB device that owns it, prints its directory
usb_dev_of() {
    local p; p=$(readlink -f "$1")
    while [ "$p" != / ] && [ ! -e "$p/idVendor" ]; do p=$(dirname "$p"); done
    [ "$p" != / ] && echo "$p"
}

echo "${B}Installed files${N}"
for pair in "99-jo-usb.rules:/etc/udev/rules.d/99-jo-usb.rules" \
            "jo-usbfs.conf:/etc/tmpfiles.d/jo-usbfs.conf" \
            "jo-cpu-idle.service:/etc/systemd/system/jo-cpu-idle.service"; do
    src="$DIR/${pair%%:*}"; dst="${pair#*:}"
    if [ ! -e "$dst" ]; then fail "$dst missing (run: sudo bash host/install_host_rules.sh)"
    elif cmp -s "$src" "$dst"; then ok "$dst"
    else fail "$dst differs from the repo copy (re-run install_host_rules.sh)"; fi
done

echo "${B}usbfs buffer${N}"
v=$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null)
[ "$v" = 1000 ] && ok "usbfs_memory_mb = 1000" || fail "usbfs_memory_mb = ${v:-?} (expected 1000)"

echo "${B}Xsens MTi (FTDI 2639:0301)${N}"
found=0
for d in /sys/bus/usb-serial/devices/*; do
    [ -e "$d" ] || continue
    u=$(usb_dev_of "$d") || continue
    [ "$(cat "$u/idVendor"):$(cat "$u/idProduct")" = 2639:0301 ] || continue
    found=1; t=$(cat "$d/latency_timer" 2>/dev/null)
    [ "$t" = 1 ] && ok "$(basename "$d") latency_timer = 1" \
                 || fail "$(basename "$d") latency_timer = ${t:-?} (expected 1: 400 Hz samples arrive in bursts)"
done
[ $found = 1 ] || fail "Xsens not connected (no usb-serial device 2639:0301)"

echo "${B}USB autosuspend (power/control = on for every device)${N}"
bad=()
for d in /sys/bus/usb/devices/*; do
    [ -e "$d/power/control" ] || continue
    [ "$(cat "$d/power/control")" = on ] || bad+=("$(basename "$d")($(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null))")
done
[ ${#bad[@]} = 0 ] && ok "all USB devices: on" || fail "autosuspend still enabled on: ${bad[*]}"

echo "${B}RealSense D456 (8086:0b5c)${N}"
declare -A seen=()
for d in /sys/bus/usb/devices/*; do
    [ "$(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null)" = 8086:0b5c ] || continue
    sn=$(cat "$d/serial" 2>/dev/null); name=${CAM[$sn]:-unknown}; seen[$name]=1
    speed=$(cat "$d/speed" 2>/dev/null); lpm=$(cat "$d/port/usb3_lpm_permit" 2>/dev/null)
    u1=$(cat "$d/power/usb3_hardware_lpm_u1" 2>/dev/null); u2=$(cat "$d/power/usb3_hardware_lpm_u2" 2>/dev/null)
    tag="$name camera ($sn, $(basename "$d"))"
    if [ "${speed:-0}" -ge 5000 ] 2>/dev/null; then ok "$tag on USB3 (${speed} Mb/s)"
    else fail "$tag on USB ${speed:-?} Mb/s: not USB3 (cable/port), the streams will not fit"; fi
    [ "$lpm" = 0 ] && ok "$tag usb3_lpm_permit = 0 (U1 ${u1:-?}, U2 ${u2:-?})" \
                   || fail "$tag usb3_lpm_permit = ${lpm:-?} (expected 0; U1 ${u1:-?}, U2 ${u2:-?})"
done
for name in front back; do [ -n "${seen[$name]}" ] || fail "$name camera not detected on USB"; done

echo "${B}CPU idle states (jo-cpu-idle.service)${N}"
st=$(systemctl is-enabled jo-cpu-idle.service 2>/dev/null); ac=$(systemctl is-active jo-cpu-idle.service 2>/dev/null)
[ "$st" = enabled ] && ok "service enabled" || fail "service ${st:-not installed} (expected enabled)"
[ "$ac" = active ] && ok "service active" || fail "service ${ac:-inactive} (expected active)"
bad=0; total=0
for s in /sys/devices/system/cpu/cpu*/cpuidle/state*; do
    [ -e "$s/latency" ] || continue
    [ "$(cat "$s/latency")" -gt 100 ] || continue
    total=$((total + 1)); [ "$(cat "$s/disable")" = 1 ] || bad=$((bad + 1))
done
if [ $total = 0 ]; then info "no idle state with exit latency > 100 us"
elif [ $bad = 0 ]; then ok "all $total idle states with exit latency > 100 us disabled"
else fail "$bad of $total idle states with exit latency > 100 us still enabled"; fi

echo
[ $FAILS = 0 ] && echo "${G}${B}All host settings are in effect.${N}" || echo "${R}${B}$FAILS check(s) failed.${N}"
exit $FAILS
