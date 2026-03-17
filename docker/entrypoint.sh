#!/usr/bin/env bash
set -e

sudo ip link set can0 up type can bitrate 500000 2>/dev/null || echo "CAN interface not found"

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select jo_bringup \
  --symlink-install \

# 3) Source overlay
source install/setup.bash
source /save_map.bash
# 4) Run whatever was passed (ros2 launch ...)
exec "$@"
