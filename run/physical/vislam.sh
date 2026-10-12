#!/usr/bin/env bash
# Basic D455 stereo visual-inertial SLAM (sparse visual map only). Extra args are passed
# to the launch file, e.g. run/physical/vislam.sh viz:=false   or   run/physical/vislam.sh localization:=true
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../.."
source /opt/ros/humble/setup.bash
exec ros2 launch "$REPO/lunabotics_slam/launch/d455_vislam.launch.py" "$@"
