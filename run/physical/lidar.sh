#!/usr/bin/env bash
# Start "simulated 3D LiDAR" mapping: D455 depth -> point cloud -> ICP odometry + SLAM.
# Extra args are passed to the launch file, e.g. run/physical/lidar.sh viz:=false
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../.."
source /opt/ros/humble/setup.bash
exec ros2 launch "$REPO/lunabotics_slam/launch/d455_lidar.launch.py" "$@"
