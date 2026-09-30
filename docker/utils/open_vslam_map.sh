#!/usr/bin/env bash
# Opens an RTAB-Map database written by `localization*.launch.py vslam:=true`
# in rtabmap-databaseViewer (graph, per-node images/scans, loop closures, 3D
# view, export). Run this INSIDE the jo-zotac container. Aliased as
# `open_vslam` in utilities.sh.
#
#   open_vslam                   # latest /saved_maps/*_rtabmap/rtabmap.db
#   open_vslam path/to/rtabmap.db
set -euo pipefail

db="${1:-$(ls -t /saved_maps/*_rtabmap/rtabmap.db 2>/dev/null | head -1 || true)}"
if [ -z "$db" ] || [ ! -f "$db" ]; then
    echo "No RTAB-Map database found (looked for /saved_maps/*_rtabmap/rtabmap.db)" >&2
    exit 1
fi

echo "Opening $db"
exec rtabmap-databaseViewer "$db"
