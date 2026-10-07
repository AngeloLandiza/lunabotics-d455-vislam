# lunabotics-d455-vislam

Stereo visual-inertial SLAM on an Intel RealSense D455. It runs entirely on stock ROS 2 Humble packages, so nothing has to be compiled:

| Stage | Package | What it does |
|---|---|---|
| Sensor | `realsense2_camera` | Publishes the left/right IR stereo pair (848x480 @ 30 Hz, hardware-synced, IR projector off) and the built-in BMI055 gyro + accel, merged into one `/camera/imu` topic at 200 Hz |
| IMU fusion | `imu_filter_madgwick` | Combines gyro and accel into an orientation estimate on `/imu/data` |
| Odometry | `rtabmap_odom/stereo_odometry` | Stereo visual odometry. It uses the IMU as the motion prior and for gravity alignment, and starts only once IMU data is available |
| SLAM | `rtabmap_slam/rtabmap` | Pose-graph SLAM with loop closure and IMU gravity constraints. It saves the map to an SQLite database |
| GUI | `rtabmap_viz` | Optional live view of the map, trajectory and features |

## Setup (once per machine)

```bash
sudo ./setup.sh   # installs any missing apt packages + RealSense udev rules
# then unplug and replug the D455
```

The udev rules are required for the IMU. Without them, the camera's HID/IIO sensor nodes stay root-only. The driver then fails with `Failed to open scan_element` / `Hid device is busy` and keeps resetting the USB device.

## Run

```bash
./run.sh                                   # map with GUI (map saved to ~/.ros/d455_vislam.db)
./run.sh viz:=false                        # headless, e.g. on the robot
./run.sh rtabmap_args:=--delete_db_on_start  # start a fresh map
./run.sh localization:=true                # localize in the saved map, no new mapping
./run.sh database_path:=$HOME/arena.db     # use a specific map file
```

Outputs:

- `/odom`: visual-inertial odometry (`odom` -> `camera_link` TF)
- `map` -> `odom` TF: drift correction from loop closures
- `/mapData`, `/mapGraph`, `/cloud_map`, `/grid_prob_map`: map products

To view a saved map offline: `rtabmap-databaseViewer ~/.ros/d455_vislam.db`

## Tips

- Keep the camera still for a second at startup. Odometry waits for the IMU to initialize gravity alignment.
- The IR emitter is turned off on purpose. Its projected dot pattern moves with the camera, which corrupts feature tracking.
- If odometry is lost (red background in the GUI), point the camera back at textured scenery. Odometry resets automatically.
- To mount on the robot, publish a static TF `base_link -> camera_link` and run with `frame_id:=base_link`.
