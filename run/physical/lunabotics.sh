#!/usr/bin/env bash
# Physical D455: the same SLAM as the sim / arena runs (lunabotics_slam/launch/lunabotics.launch.py):
#   visual-inertial tracking + ICP-refined dense map + terrain height map (dig/dump volume)
#   + boulder / crater segmentation for the driver.
# Extra args are passed to the launch file, e.g.
#   run/physical/lunabotics.sh viz:=false   or   run/physical/lunabotics.sh localization:=true
# The older sparse stereo VISLAM is run/physical/vislam.sh.
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../.."
source /opt/ros/humble/setup.bash
exec ros2 launch "$REPO/lunabotics_slam/launch/lunabotics.launch.py" "$@"
