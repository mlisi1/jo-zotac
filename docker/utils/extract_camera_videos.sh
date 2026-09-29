#!/usr/bin/env bash
# Thin wrapper around extract_camera_videos.py. Run this INSIDE the jo-zotac
# container -- it needs rosbag2_py, cv_bridge, opencv and ffmpeg, all
# available there but not on the host. Aliased as `extract_videos` in
# utilities.sh, so you normally don't call this file directly.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/extract_camera_videos.py" "$@"
