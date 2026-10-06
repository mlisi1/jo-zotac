# Bag recording helpers. Topic lists, QoS and storage settings live in
# docker/utils/recording/*.yaml (mounted at /home/ros/utils/recording).
_REC_CFG=/home/ros/utils/recording
_REC_DEST=/home/ros/bags

_record() {  # _record <profile> <name> [extra --ros-args ...]
    local profile="$1" name="$2"; shift 2
    mkdir -p "$_REC_DEST"
    ros2 run rosbag2_transport recorder --ros-args \
        --params-file "${_REC_CFG}/${profile}.yaml" \
        -p storage.uri:="${_REC_DEST}/$(date +%Y%m%d_%H%M%S)_${name}" \
        "$@"
}

record_all() {
    if [ $# -eq 0 ]; then echo "Usage: record_all <name> [ros-args]"; return 1; fi
    _record record_all "$@"
}

record_compressed() {
    if [ $# -eq 0 ]; then echo "Usage: record_compressed <name> [ros-args]"; return 1; fi
    _record record_compressed "$@"
}

record_sensors() {
    if [ $# -eq 0 ]; then echo "Usage: record_sensors <name> [ros-args]"; return 1; fi
    _record record_sensors "$@"
}

# Calibration recordings (profiles recording/calib_*.yaml, see CALIBRATION.md).
# Bags are named <timestamp>_calib_<profile>_<name>.
record_calib() {
    local profiles
    profiles=$(cd "$_REC_CFG" && ls calib_*.yaml 2>/dev/null | sed 's/^calib_//; s/\.yaml$//' | tr '\n' ' ')
    if [ $# -lt 2 ] || [ ! -f "${_REC_CFG}/calib_$1.yaml" ]; then
        echo "Usage: record_calib <profile> <name> [ros-args]"
        echo "Profiles: ${profiles}"
        return 1
    fi
    local profile="$1" name="$2"; shift 2
    _record "calib_${profile}" "calib_${profile}_${name}" "$@"
}
