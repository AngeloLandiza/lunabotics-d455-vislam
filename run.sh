#!/usr/bin/env bash
# Physical D455: the same SLAM as the sim / arena runs (launch/lunabotics.launch.py):
#   visual-inertial tracking + ICP-refined dense map + terrain height map (dig/dump volume)
#   + boulder / crater segmentation for the driver.
# Extra args are passed to the launch file, e.g. ./run.sh viz:=false   or   ./run.sh localization:=true
# The older sparse stereo VISLAM is ./run_vislam.sh.
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/lunabotics.launch.py" "$@"
