#!/usr/bin/env bash
# Exports a coloured point cloud (.ply) from an RTAB-Map database written by
# `localization*.launch.py vslam:=true`. Run this INSIDE the jo-zotac
# container after the mapping run has been stopped (rtabmap saves the
# database on shutdown). Aliased as `export_vslam` in utilities.sh.
#
#   export_vslam [options] [db]  # db defaults to the latest /saved_maps/*_rtabmap/rtabmap.db
#
# Options:
#   --rgbd          camera depth cloud instead of lidar
#   --voxel M       voxel size in metres; one point is kept per voxel, so
#                   bigger = smaller file (default 0.05 lidar, 0.02 rgbd;
#                   0 = no downsampling)
#   --decimation N  --rgbd only: use every Nth depth pixel in each direction
#                   before building the cloud (default 4; 1 = full resolution)
#
# Default: Velodyne scans assembled on the optimized graph, each point
# coloured by projecting it into the front camera images. Only cameras within
# 4 m of a point may colour it, and the closest one wins. Without this, far
# floor points (seen at grazing angles from distant cameras) take the colour
# of whatever stood in front of them, giving long single-colour streaks.
# Points no camera saw within 4 m are dropped (~15%).
# --rgbd builds the cloud from the D456 depth images instead (what the
# databaseViewer shows): colour is pixel-aligned, denser up close, but
# limited to 4 m.
# Output: <db dir>/rtabmap_cloud.ply
set -euo pipefail

usage() { sed -n '7,15p' "$0" | sed 's/^# \{0,1\}//'; }

mode=lidar
voxel=""
decimation=4
db=""
while [ $# -gt 0 ]; do
    case "$1" in
        --rgbd)       mode=rgbd; shift ;;
        --voxel)      voxel="${2:?--voxel needs a value in metres}"; shift 2 ;;
        --decimation) decimation="${2:?--decimation needs an integer}"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        -*)           echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *)            db="$1"; shift ;;
    esac
done

if ! [[ "$decimation" =~ ^[1-9][0-9]*$ ]]; then
    echo "--decimation must be a positive integer, got '$decimation'" >&2
    exit 1
fi
if [ -n "$voxel" ] && ! [[ "$voxel" =~ ^[0-9]*\.?[0-9]+$ ]]; then
    echo "--voxel must be a non-negative number in metres, got '$voxel'" >&2
    exit 1
fi

if [ -z "$db" ]; then
    db="$(ls -t /saved_maps/*_rtabmap/rtabmap.db 2>/dev/null | head -1 || true)"
fi
if [ -z "$db" ] || [ ! -f "$db" ]; then
    echo "No RTAB-Map database found (looked for /saved_maps/*_rtabmap/rtabmap.db)" >&2
    exit 1
fi

if [ "$mode" = lidar ]; then
    voxel="${voxel:-0.05}"
    args=(--scan --cam_projection --voxel "$voxel" --texture_range 4 --texture_d2c)
    settings="voxel ${voxel} m"
else
    voxel="${voxel:-0.02}"
    args=(--max_range 4 --voxel "$voxel" --decimation "$decimation")
    settings="voxel ${voxel} m, decimation ${decimation}"
fi

echo "Exporting $mode cloud ($settings) from $db"
rtabmap-export --cloud "${args[@]}" \
    --output rtabmap --output_dir "$(dirname "$db")" "$db"
echo "Saved: $(dirname "$db")/rtabmap_cloud.ply"
