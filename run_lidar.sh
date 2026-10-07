#!/usr/bin/env bash
# Start "simulated 3D LiDAR" mapping: D455 depth -> point cloud -> ICP odometry + SLAM.
# Extra args are passed to the launch file, e.g. ./run_lidar.sh viz:=false
set -eo pipefail
source /opt/ros/humble/setup.bash
exec ros2 launch "$(dirname "$(readlink -f "$0")")/launch/d455_lidar.launch.py" "$@"
