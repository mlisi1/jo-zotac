# save_map <name>: save the GLIM map to /saved_maps/<YYYYmmdd_HHMMSS>_<name>/
#
# GLIM running: calls its ~/save_map service (glim_ros2@dev), which saves while SLAM keeps running:
#   GLIM dump (graph, submaps: opens in offline_viewer), map.pcd, map.pgm/map.yaml (nav2_map_server)
#   and map_anchor.txt. The service saves in a background thread and returns at once, so this waits
#   until map.yaml is written and the folder is quiet, then adds <name> to the folder.
# GLIM stopped: copies /saved_maps/glim_last_run, which GLIM writes at shutdown (same files). It holds
#   the LAST run only: the next GLIM start clears it (jo_navigation launch/glim_common.py).
save_map() {
    if [ $# -eq 0 ]; then
        echo "Usage: save_map <name>"
        return 1
    fi
    local name="$1" dest_dir="/saved_maps" ros2_bin
    ros2_bin="$(type -P ros2)"   # the executable: `ros2` is DendROS's shell function
    mkdir -p "$dest_dir"

    if timeout 8 "$ros2_bin" service type /glim_ros/save_map >/dev/null 2>&1; then
        local out path
        out=$(timeout 15 "$ros2_bin" service call /glim_ros/save_map std_srvs/srv/Trigger 2>&1)
        if ! grep -q "success=True" <<< "$out"; then
            echo "Error: GLIM refused to save: $(sed -n "s/.*message='\([^']*\)'.*/\1/p" <<< "$out")"
            return 1
        fi
        path=$(sed -n "s/.*message='\([^']*\)'.*/\1/p" <<< "$out")
        echo "GLIM is saving to $path (SLAM keeps running) ..."
        # Wait for map.yaml (written last) and 2 s without changes; up to 10 min for large maps
        local waited=0 last="" now
        while [ $waited -lt 600 ]; do
            sleep 1; waited=$((waited + 1))
            [ -f "$path/map.yaml" ] || continue
            now=$(find "$path" -type f -printf '%s %T@\n' 2>/dev/null | sort | md5sum)
            if [ "$now" = "$last" ]; then
                mv "$path" "${path}_${name}" && echo "Map saved to: ${path}_${name}" && return 0
                echo "Error: could not rename $path"; return 1
            fi
            last="$now"
        done
        echo "Warning: no map.yaml after ${waited}s (no occupancy grid?): left as $path"
        return 1
    fi

    # GLIM not running: the shutdown dump
    local source_path="${dest_dir}/glim_last_run"
    if [ ! -d "$source_path" ]; then
        echo "Error: GLIM is not running and there is no $source_path (GLIM never ran with this /saved_maps, or a new GLIM run is starting)"
        return 1
    fi
    echo "GLIM is not running: copying its shutdown dump $source_path (written $(date -r "$source_path" '+%F %T'))"
    local dest_path="${dest_dir}/$(date -r "$source_path" +%Y%m%d_%H%M%S)_${name}"   # the run's time
    if cp -r "$source_path" "$dest_path"; then
        echo "Map saved to: $dest_path"
    else
        echo "Error: Failed to save map"
        return 1
    fi
}
