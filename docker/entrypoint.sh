#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select jo_bringup \
  --symlink-install \

# 3) Source overlay
source install/setup.bash

# 4) Run whatever was passed (ros2 launch ...)
exec "$@"
