#!/usr/bin/env bash
# Lunabotics terrain SLAM: fusion localization + semi-dynamic terrain map (dig/berm volume).
# Extra args are passed to the launch file, e.g. ./run_lunabotics.sh localization:=true
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/lunabotics.launch.py" "$@"
