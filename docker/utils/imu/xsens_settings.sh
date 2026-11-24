#!/bin/bash
# Build (into /tmp, a few seconds) and run xsens_settings.cpp against the driver's XDA library.
# Stop the Xsens driver first. No arguments = read only.
set -e
X=/home/ros/src/Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/xsens_mti_ros2_driver/lib/xspublic
BIN=/tmp/xsens_settings
SRC="$(dirname "$(readlink -f "$0")")/xsens_settings.cpp"
if [[ ! -x $BIN || $SRC -nt $BIN ]]; then
    g++ -O2 -std=c++17 -I"$X" "$SRC" -o "$BIN" -L"$X/xscontroller" -L"$X/xscommon" -L"$X/xstypes" \
        -lxscontroller -lxscommon -lxstypes -lpthread -ldl -no-pie
fi
exec "$BIN" "$@"
