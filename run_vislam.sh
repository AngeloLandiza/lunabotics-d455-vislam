#!/usr/bin/env bash
# Basic D455 stereo visual-inertial SLAM (sparse visual map only). Extra args are passed
# to the launch file, e.g. ./run_vislam.sh viz:=false   or   ./run_vislam.sh localization:=true
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/d455_vislam.launch.py" "$@"
