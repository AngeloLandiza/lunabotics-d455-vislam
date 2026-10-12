#!/usr/bin/env bash
# Colour 3D reconstruction with the physical D455: scan with RGB-D SLAM, then on Ctrl+C
# export the scan as a textured mesh + coloured point cloud (run/export_mesh.sh).
#
#   run/physical/reconstruct.sh                scan (saved to ~/.ros/d455_reconstruct.db), export on exit
#   run/physical/reconstruct.sh viz:=false     headless
#   run/physical/reconstruct.sh --no-export    scan only; export later with run/export_mesh.sh
#
# Other args are passed to lunabotics_slam/launch/d455_reconstruct.launch.py. Move the camera slowly and
# revisit areas from a few angles: every keyframe is a texture source.
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../.."
db="$HOME/.ros/d455_reconstruct.db"; export=1; launch_args=()
for a in "$@"; do
  case "$a" in
    --no-export) export=0 ;;
    database_path:=*) db="${a#database_path:=}"; launch_args+=("$a") ;;
    *) launch_args+=("$a") ;;
  esac
done

source /opt/ros/humble/setup.bash
trap : INT   # Ctrl+C stops the launch (rtabmap saves the map), not this script
ros2 launch "$REPO/lunabotics_slam/launch/d455_reconstruct.launch.py" database_path:="$db" "${launch_args[@]}" || true
trap - INT

if [ "$export" = 1 ]; then
  exec "$REPO/run/export_mesh.sh" "$db"
fi
