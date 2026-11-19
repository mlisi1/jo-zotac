#!/usr/bin/env bash
set -e

# 1) CAN for the Bunker base (absent when the adapter isn't plugged in, e.g. off the robot)
sudo ip link set can0 up type can bitrate 500000 2>/dev/null || echo "CAN interface not found"

# 2) Build the mounted packages (jo_bringup, jo_description, jo_navigation)
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select jo_bringup jo_description jo_navigation \
  --symlink-install

# 3) Source the overlay. Aliases and helpers (save_map, record_*) come from utilities.sh via .bashrc:
#    shell functions wouldn't survive the exec below anyway
source install/setup.bash

# 4) Run whatever was passed (bash, ros2 launch ...)
exec "$@"
