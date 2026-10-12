# lunabotics_slam

The arena SLAM stack as a ROS 2 (Humble) package, for the real Lunabotics robot with an Intel RealSense D455. It's the same pipeline the arena sim runs (`run/sim/arena/arena.sh`):

- **Fusion SLAM:** stereo visual-inertial odometry on projector-off IR, refined by ICP on projector-on dense depth (RTAB-Map).
- **Terrain map:** a 2.5D height map in the SLAM map frame that re-maps dug and dumped areas and reports dug (cut) and dumped (fill) volume.
- **Obstacle segmentation:** boulders, craters, dug pits, piles, walls and the pillar, as labelled RViz markers plus an occupancy grid for path planning.

Everything is stock ROS 2 Humble packages plus four Python nodes; nothing is compiled.

## Install on the robot

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repo>                       # colcon finds the package in lunabotics_slam/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # realsense2_camera, rtabmap_ros, imu_filter_madgwick, ...
colcon build --packages-select lunabotics_slam
source install/setup.bash

# Once per machine: RealSense udev rules (the D455 IMU needs them), then replug the camera
sudo install -m 0644 install/lunabotics_slam/share/lunabotics_slam/udev/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add `source ~/ros2_ws/install/setup.bash` to `~/.bashrc` on the robot.

## Run

```bash
# Survey: map the arena + record the terrain baseline (camera on the robot body)
ros2 launch lunabotics_slam lunabotics.launch.py base_frame:=base_link \
    camera_mount:="0.30 0 0.42 0 0.70 0" viz:=false

# Competition runs: localize in the surveyed map, load the terrain, report dug volume
ros2 launch lunabotics_slam lunabotics.launch.py base_frame:=base_link \
    camera_mount:="0.30 0 0.42 0 0.70 0" viz:=false localization:=true
```

Other modes in the package: `d455_fusion.launch.py` (SLAM only), `d455_vislam.launch.py`, `d455_lidar.launch.py`, `d455_reconstruct.launch.py`. Run `ros2 launch lunabotics_slam <file> --show-args` to list each one's options.

## Robot settings

| Argument | Default | What it does |
|---|---|---|
| `base_frame` | `camera_link` | Robot frame that odometry and SLAM report (`odom` → `base_frame`). Use `base_link` on the robot. |
| `camera_mount` | empty | `camera_link` pose in `base_frame`: `"x y z roll pitch yaw"` in m and rad (pitch > 0 = tilted down). Publishes the static TF. Leave it empty if your robot description (URDF / `robot_state_publisher`) already publishes it. |
| `odom_guess_frame` | empty | A wheel-odometry frame (e.g. from `robot_localization`) to use as odometry's motion guess. Empty = IMU only. |
| `viz` | `true` | RViz + the camera/landmarks window. Use `false` on the robot and view remotely. |
| `localization` | `false` | Localize in the saved map instead of mapping. |
| `database_path` | `~/.ros/lunabotics.db` | Map file (survey writes it, runs read it). |
| `terrain_file` | `~/.ros/lunabotics_terrain.npz` | Terrain map + baseline. |
| `config`, `terrain_config` | package `config/` | Parameter files; copy and pass your own to tune. |

`camera_link` sits at the D455's left IR camera, with x forward, y left and z up. The example mount (0.30 m forward, 0.42 m up, pitched 40° = 0.70 rad down) is the sim rover's; measure your own.

## Topics

| Topic | Type | Content |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Visual-inertial odometry (`odom` → `base_frame` TF) |
| TF `map` → `odom` | | SLAM drift correction |
| `/cloud_map` | `sensor_msgs/PointCloud2` | Dense 3D map |
| `/terrain/cloud`, `/terrain/elevation` | `PointCloud2`, `Image` | Height map: elevation, change since the baseline, variance per 5 cm cell |
| `/terrain/volume` | `std_msgs/Float64MultiArray` | `[cut, fill]` m³ (dug, dumped) |
| `/terrain/obstacles` | `visualization_msgs/MarkerArray` | Labelled boulders, craters, pits, piles, walls, pillar |
| `/terrain/obstacle_grid` | `nav_msgs/OccupancyGrid` | 100 = obstacle, 0 = known free, -1 = unknown: feed it to your planner |

Services: `/terrain/reset_baseline`, `/terrain/clear` (`std_srvs/Trigger`).

## Settings

- `config/d455_fusion.yaml`: camera, IMU filter, odometry and RTAB-Map. One section per node, every setting commented.
- `config/lunabotics_terrain.yaml`: the terrain map and obstacle segmentation (thresholds, grid size, ranges).

The repo's main README explains how each part works and what was measured in the arena sim.
