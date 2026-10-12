#!/usr/bin/env bash
# Start fusion mapping: visual-inertial tracking + ICP-refined dense map (most accurate).
# Extra args are passed to the launch file, e.g. run/physical/fusion.sh viz:=false
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../.."
source /opt/ros/humble/setup.bash
exec ros2 launch "$REPO/lunabotics_slam/launch/d455_fusion.launch.py" "$@"
