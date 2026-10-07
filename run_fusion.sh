#!/usr/bin/env bash
# Start fusion mapping: visual-inertial tracking + ICP-refined dense map (most accurate).
# Extra args are passed to the launch file, e.g. ./run_fusion.sh viz:=false
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/d455_fusion.launch.py" "$@"
