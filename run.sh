#!/usr/bin/env bash
# Start D455 stereo visual-inertial SLAM. Extra args are passed to the launch file,
# e.g. ./run.sh viz:=false   or   ./run.sh localization:=true
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/d455_vislam.launch.py" "$@"
