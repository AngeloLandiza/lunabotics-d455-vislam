# lunabotics-d455-vislam

Stereo visual-inertial SLAM on an Intel RealSense D455. It runs entirely on stock ROS 2 Humble packages, so nothing has to be compiled:

| Stage | Package | What it does |
|---|---|---|
| Sensor | `realsense2_camera` | Publishes the left/right IR stereo pair (848x480 @ 30 Hz, hardware-synced, IR projector off, 5 ms fixed exposure) and the built-in BMI055 gyro + accel, merged into one `/camera/imu` topic at 200 Hz |
| IMU fusion | `imu_filter_madgwick` | Combines gyro and accel into an orientation estimate on `/imu/data` |
| Odometry | `rtabmap_odom/stereo_odometry` | Stereo visual odometry. It uses the IMU as the motion prior and for gravity alignment, and resets itself automatically if tracking is lost |
| SLAM | `rtabmap_slam/rtabmap` | Pose-graph SLAM with loop closure and IMU gravity constraints. It builds the dense map from the stereo pair (SGBM) and saves it to an SQLite database |
| Views | `rviz2` + `scripts/vislam_viewer.py` | Opens two clean windows (described below) |

## Setup (once per machine)

```bash
sudo ./setup.sh   # installs any missing apt packages + RealSense udev rules
# then unplug and replug the D455
```

The udev rules are required for the IMU. Without them, the camera's HID/IIO sensor nodes stay root-only. The driver then fails with `Failed to open scan_element` / `Hid device is busy` and keeps resetting the USB device.

## Run

```bash
./run.sh                                     # map with both windows (map saved to ~/.ros/d455_vislam.db)
./run.sh viz:=false                          # headless, e.g. on the robot
./run.sh rtabmap_args:=--delete_db_on_start  # start a fresh map
./run.sh localization:=true                  # localize in the saved map, no new mapping
./run.sh database_path:=$HOME/arena.db       # use a specific map file
./run.sh ir_gain:=120                        # brighter IR image in a dim room
./run.sh rtabmap_viz:=true                   # also open the stock RTAB-Map GUI
```

### Windows

- **Map + path (RViz):** the dense map, coloured by height, on a dark background. It also shows the smooth full-rate trajectory in pink and the camera frustum in cyan. Use the mouse to orbit, pan and zoom. To show the side panels, open the *Panels* menu. To show the 3D odometry landmarks, enable *Live landmarks* in the Displays panel.
- **Camera + landmarks (OpenCV):** the left IR image with every tracked feature drawn on it:
  - **Green with trail:** inlier, used in the pose estimate.
  - **Amber ring:** matched but rejected as an outlier.
  - **Grey dot:** detected but unmatched.

  The top bar shows tracking state, inlier and feature counts, odometry rate, latency and distance travelled. The frame turns red when tracking is lost. Press `q` to close it.

The annotated camera image is also published on `/vislam/feature_image`, so you can view it remotely (e.g. in RViz on a laptop while the robot runs `viz:=false`).

### Topics

- `/odom`: visual-inertial odometry (`odom` -> `camera_link` TF)
- `map` -> `odom` TF: drift correction from loop closures
- `/cloud_map`, `/grid_prob_map`, `/mapData`, `/mapGraph`: map products
- `/vislam/trajectory`: full-rate path for display

To view a saved map offline: `rtabmap-databaseViewer ~/.ros/d455_vislam.db`

## Tuning notes

- **Exposure:** IR auto-exposure picks 15–20 ms indoors with the projector off. That motion-blurs features and loses tracking. The firmware ignores `auto_exposure_limit`, so the launch file fixes exposure at 5 ms and raises gain to 64 to keep the same brightness. Raise `ir_gain` in a dim room. Set `auto_exposure:=true` to go back to auto.
- **IR projector:** it's off on purpose. Its dot pattern moves with the camera, which corrupts feature tracking.
- **Timing:** IR and IMU timestamps both use librealsense `global_time`, meaning hardware time corrected to the host clock. Measured on this unit:
  - The stereo pair has identical timestamps on every frame.
  - The IMU runs at 200 Hz with 5.01 ms spacing.
  - No dropped or out-of-order samples.
  - Both streams drift the same ~0.2 ms/min relative to the host, so they stay in sync with each other.

  The startup warning `frame's time domain is HARDWARE_CLOCK` comes from the very first frames only, before the host-clock fit has enough samples.
- **Startup:** keep the camera still for a second at startup. Odometry waits for the IMU to initialize gravity alignment.
- **Robot mounting:** publish a static TF `base_link -> camera_link` and run with `frame_id:=base_link`.
