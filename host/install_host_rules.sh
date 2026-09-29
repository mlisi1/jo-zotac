#!/usr/bin/env bash
# Installs the host-side USB/CPU settings (udev rules, usbfs limit, CPU idle
# service) and applies
# them to already-connected devices. Run on the host (not in the container):
#   sudo bash host/install_host_rules.sh
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"

install -m 644 "$DIR/99-jo-usb.rules" /etc/udev/rules.d/99-jo-usb.rules
install -m 644 "$DIR/jo-usbfs.conf" /etc/tmpfiles.d/jo-usbfs.conf
install -m 644 "$DIR/jo-cpu-idle.service" /etc/systemd/system/jo-cpu-idle.service

udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb --subsystem-match=usb-serial
udevadm settle
systemd-tmpfiles --create /etc/tmpfiles.d/jo-usbfs.conf
systemctl daemon-reload
systemctl enable --now jo-cpu-idle.service

echo "usbfs_memory_mb: $(cat /sys/module/usbcore/parameters/usbfs_memory_mb)"
for d in /sys/bus/usb-serial/devices/*; do
    [ -e "$d/latency_timer" ] && echo "$(basename "$d") latency_timer: $(cat "$d/latency_timer")"
done
echo "USB power/control: $(cat /sys/bus/usb/devices/*/power/control | sort | uniq -c | tr '\n' ' ')"
echo "CPU idle states disabled on cpu0: $(for s in /sys/devices/system/cpu/cpu0/cpuidle/state*; do [ "$(cat $s/disable)" = 1 ] && printf "%s " "$(cat $s/name)"; done)  (jo-cpu-idle.service: $(systemctl is-enabled jo-cpu-idle.service))"
