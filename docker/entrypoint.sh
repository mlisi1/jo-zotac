#!/usr/bin/env bash
set -e

sudo ip link set can0 up type can bitrate 500000 2>/dev/null || echo "CAN interface not found"

source /opt/ros/jazzy/setup.bash

# Incremental build of the volume-mounted packages. glim/glim_ros are NOT
# rebuilt here (too heavy): after editing them run `build_glim`.
# -DCUDAToolkit_ROOT: see the onboard_detector_v2 note in the Dockerfile.
colcon build \
  --packages-select jo_msgs jo_bringup jo_description jo_navigation onboard_detector_v2 \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDAToolkit_ROOT=${CUDAToolkit_ROOT}

# 3) Source overlay
source install/setup.bash
source /home/ros/utils/save_map.bash
# 4) Run whatever was passed (ros2 launch ...)
exec "$@"
