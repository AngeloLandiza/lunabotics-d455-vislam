# lunabotics-d455-vislam

Stereo visual-inertial SLAM on an Intel RealSense D455, plus a Lunabotics terrain map with boulder and crater detection. It runs entirely on stock ROS 2 Humble packages, so nothing has to be compiled.

```bash
./run.sh                      # the physical D455: the full Lunabotics pipeline, same as the sim and arena runs
./run.sh localization:=true   # competition runs: localize in the surveyed map
./run.sh viz:=false           # headless, e.g. on the robot
```

`./run.sh` runs [Lunabotics terrain mode](#lunabotics-terrain-mode-semi-dynamic-map-for-digging):
- [fusion](#fusion-mode-most-accurate) tracking, i.e. visual-inertial odometry refined by ICP on dense depth;
- the terrain height map;
- [boulder / crater segmentation](#boulder-and-crater-segmentation).

The basic stereo VISLAM pipeline below is still available as `./run_vislam.sh`:

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

## Basic stereo VISLAM (`./run_vislam.sh`)

```bash
./run_vislam.sh                              # new map with both windows (saved to ~/.ros/d455_vislam.db)
./run_vislam.sh viz:=false                   # headless, e.g. on the robot
./run_vislam.sh new_map:=false               # extend the saved map instead of starting over
./run_vislam.sh localization:=true           # localize in the saved map, no new mapping
./run_vislam.sh database_path:=$HOME/arena.db # use a specific map file
./run_vislam.sh dynamic_map:=true            # map forgets surfaces that are gone (e.g. dug terrain)
./run_vislam.sh config:=config/my_room.yaml  # use a different settings file
./run_vislam.sh rtabmap_viz:=true            # also open the stock RTAB-Map GUI
```

### Settings

All camera, IMU, odometry and SLAM settings live in [config/d455_vislam.yaml](config/d455_vislam.yaml), one section per node, each setting commented. Edit it and rerun; nothing needs rebuilding. For a variant, such as a dim room with `depth_module.gain: 160`, copy the file and pass it with `config:=`. The launch flags above only pick modes.

`dynamic_map:=true` loads [config/dynamic_map.yaml](config/dynamic_map.yaml) on top of the main settings. That file turns on ray tracing in RTAB-Map's 3D voxel map (OctoMap), so voxels the camera can now see through, such as dug-out ground, are removed. RViz then shows `/octomap_occupied_space` instead of `/cloud_map`.

### Windows

- **Map + path (RViz):** the dense map, coloured by height, on a dark background. It also shows the smooth full-rate trajectory in pink and the camera frustum in cyan. Use the mouse to orbit, pan and zoom. To show the side panels, open the *Panels* menu. To show the 3D odometry landmarks, enable *Live landmarks* in the Displays panel.
- **Camera + landmarks (OpenCV):** the left IR image with every tracked feature drawn on it:
  - **Green with trail:** inlier, used in the pose estimate.
  - **Amber ring:** matched but rejected as an outlier.
  - **Grey dot:** detected but unmatched.

  The top bar shows tracking state, inlier and feature counts, odometry rate, latency and distance travelled. The frame turns red when tracking is lost. Press `q` or Esc to close it.

The annotated camera image is also published on `/vislam/feature_image`, so you can view it remotely (e.g. in RViz on a laptop while the robot runs `viz:=false`).

### Topics

- `/odom`: visual-inertial odometry (`odom` -> `camera_link` TF)
- `map` -> `odom` TF: drift correction from loop closures
- `/cloud_map`, `/grid_prob_map`, `/mapData`, `/mapGraph`: map products
- `/vislam/trajectory`: full-rate path for display

To view a saved map offline: `rtabmap-databaseViewer ~/.ros/d455_vislam.db`

## Simulated 3D LiDAR mode

```bash
./run_lidar.sh                    # map with RViz (saved to ~/.ros/d455_lidar.db)
./run_lidar.sh viz:=false         # headless
./run_lidar.sh localization:=true # localize in the saved LiDAR map
```

This mode treats the D455 like a 3D LiDAR. Its depth stream is converted into a point cloud on `/lidar/points`: about 8,500 points per scan at 30 Hz, 5 cm spacing, 0.3–6 m range. Mapping then runs RTAB-Map's own 3D-LiDAR pipeline (`rtabmap_examples/lidar3d.launch.py`): ICP scan-to-map odometry with the IMU as the rotation prior, then scan-based SLAM. No visual features are used, so the IR projector is on for dense depth. RViz shows the voxel map coloured by height, the live scan in white, the trajectory and the camera frustum.

To swap in a real LiDAR later, publish its cloud on `/lidar/points` and drop the camera and `point_cloud_xyz` nodes. Settings live in [config/d455_lidar.yaml](config/d455_lidar.yaml).

ICP needs geometry to lock onto. A flat floor plus one wall, or a scene with nothing within 6 m, can make it slide. The visual mode (`./run_vislam.sh`) is the more robust tracker; this mode gives the denser, LiDAR-style map.

## Fusion mode (most accurate)

```bash
./run_fusion.sh                     # map with both windows (saved to ~/.ros/d455_fusion.db)
./run_fusion.sh viz:=false          # headless
./run_fusion.sh localization:=true  # localize in the saved fusion map
```

This mode combines the strengths of the other two using one D455:

1. The IR projector switches on every other frame (`depth_module.emitter_on_off`). [scripts/emitter_splitter.py](scripts/emitter_splitter.py) reads each frame's `frame_emitter_mode` metadata and splits the stream by projector state:
   - Projector-off IR pairs go to `/split/infra*`. They feed stereo visual-inertial odometry, the robust tracker from `./run_vislam.sh`, at 15 Hz.
   - Projector-on depth goes to `/split/depth`. It becomes a dense point cloud on `/lidar/points`.
2. RTAB-Map uses the visual odometry as the motion guess. ICP on the dense clouds then refines every new keyframe link (`RGBD/NeighborLinkRefining`), and adds local loop closures by matching nearby scans (`RGBD/ProximityBySpace`). Visual loop closures are refined with ICP too (`Reg/Strategy 2`). The map is built from the dense clouds.
3. Where a scan is too flat to constrain every direction (a wall plus floor), ICP corrects only the constrained axes and keeps the visual estimate for the rest (`Icp/PointToPlaneLowComplexityStrategy`). That's RTAB-Map's answer to ICP degeneracy, and it covers the weakness of the LiDAR mode.

Measured on this D455:
- 15 Hz on each half of the split.
- Every ICP refinement was accepted, with an average inlier ratio of 0.34.
- About 0.3 s per map update, even with a keyframe on every update.

Settings are in [config/d455_fusion.yaml](config/d455_fusion.yaml). The next accuracy step is a wheel odometry + IMU guess, from `robot_localization` on the robot.

## Lunabotics terrain mode (semi-dynamic map for digging)

```bash
./run.sh                      # survey: map the arena + record the terrain baseline
./run.sh localization:=true   # runs: localize in that map, load the terrain, report dug volume
```

`./run_lunabotics.sh` is the same command under its old name.

This mode runs fusion localization (above) plus [scripts/terrain_mapper.py](scripts/terrain_mapper.py). The mapper keeps a 2.5D height map (5 cm cells) that **re-maps areas that change**, such as a hole being dug or a berm being built. Each cell is updated with these existing techniques:
- **Height fusion:** Kalman fusion of height measurements, with noise that grows with range² (Fankhauser et al. 2018, ETH `elevation_mapping`).
- **Change detection:** a measurement more than 3σ from the stored height, on two consecutive scans, resets the cell to the new height instead of averaging.
- **Visibility cleanup:** cells the camera now sees *through*, like the old surface over a fresh hole, are cleared (`elevation_mapping_cupy`).
- **Height-drift compensation:** each scan's vertical offset against the map is removed first, so SLAM height drift isn't mistaken for digging (`elevation_mapping_cupy`).

Height minus a baseline gives dug (cut) and dumped (fill) volume; berm volume is what Lunabotics scores.

Outputs:
- `/terrain/cloud`: one point per cell, with elevation, change and variance fields.
- `/terrain/elevation`: a 32FC1 grid image.
- `/terrain/volume`: `[cut, fill]` in m³.
- Services `/terrain/reset_baseline` and `/terrain/clear`.

### Boulder and crater segmentation

[scripts/obstacle_segmenter.py](scripts/obstacle_segmenter.py) runs on the terrain map twice a second. It shows the driver where the obstacles are and gives a planner an occupancy grid.

How it works:
1. **Local ground:** a 1 m window around each cell, taken as a low percentile and then a median with bumps and pits removed. Slopes, ducts and crater rims don't shift it.
2. **Threshold:** cells more than 8 cm above the local ground are bumps; cells more than 6 cm below it are pits.
3. **Blobs:** a morphological open removes single-cell noise, then connected blobs are sized and labelled:

| Label | RViz colour | Rule |
|---|---|---|
| boulder | orange | bump 12–90 cm across |
| crater | blue | pit 15–110 cm across |
| dug pit | cyan | pit the robot dug (terrain change < −4 cm since the baseline) |
| pile | yellow | bump the robot dumped (change > +4 cm), e.g. the berm |
| structure | grey | long, large or taller than 60 cm: ducts, the column box |

Outputs:
- `/terrain/obstacles`: MarkerArray, a cylinder per obstacle plus a label such as "boulder 34 cm". It is shown in RViz as *Boulders + craters*.
- `/terrain/obstacle_grid`: OccupancyGrid. Boulders, craters and structures are 100, other known cells 0, unseen cells −1. It is shown in RViz as *Obstacle grid* (off by default).

Offline test on 5 simulated arena rounds:
- found 30 of 31 boulders and 15 of 16 craters;
- 1 false positive, at the edge of unseen ground;
- median centre error 1.3 cm;
- about 110 ms per update.

An obstacle has to be seen before it can be segmented. The camera looks about 0.5–3.5 m ahead.

Settings are in the `obstacle_segmenter` section of [config/lunabotics_terrain.yaml](config/lunabotics_terrain.yaml).

### Terrain settings

Settings are in [config/lunabotics_terrain.yaml](config/lunabotics_terrain.yaml). The two-step workflow is the arena version of anchoring to a prior map, which is how the [DEM-anchored lunar SLAM paper](https://arxiv.org/abs/2603.17229) limits drift.

## Lunabotics arena simulation + dig tool

```bash
./run_arena.sh                    # arena + SLAM + dig tool; drive with W A S D in the dig tool window
./run_arena.sh --survey           # the rover drives survey loops on its own
./run_arena.sh --seed 4           # another round: new boulders, craters and start pose
./run_arena.sh --start-heading N  # start facing N/E/S/W (default: random, like the direction wheel)
./run_arena.sh --eval --headless  # check SLAM vs. ground truth: survey lap, dig + dump, second lap, report
```

This is a recreation of the Artemis Arena at KSC, built from the 2027 guidebook (sec. 11) plus NASA photos and 2024–2026 run footage. [sim/arena.py](sim/arena.py) holds the layout, [sim/arena_scene.py](sim/arena_scene.py) builds it in Isaac Sim.

**Bed and zones (guidebook):**
- 6.88 × 5.0 m of BP-1 between 17 cm PVC perimeter ducts, 45 cm deep.
- Zones:
  - starting zone: 2 × 2 m corner;
  - excavation zone: the first 2.5 m;
  - obstacle zone: the remaining 4.38 m;
  - construction zone: a 1.5 m strip along the far end.
- Berm target box: 2.2 × 0.9 m centred at (5.38, 0.9).

**Obstacles:** re-drawn every round from `--seed`.
- Boulders 30–40 cm, at least 3 in the obstacle zone and some in the excavation zone.
- Craters at most 50 cm wide, with a raised rim of the dug-out material.
- Nothing in the starting or construction zones, and a robot-sized path to the berm is guaranteed.

**Start pose:** random position in the starting zone, heading N/E/S/W.

**Looks (footage):**
- Light tan-grey BP-1 with old track prints, small mounds and faint rake lines.
- Light tan boulders.
- Beige bin walls about 0.45 m tall, with grey ducts at their base.
- A black-framed glass enclosure on the walls.
- A white box truss on a grid; the central leg stands in a grey box, which is the guidebook's support column.
- An 80/20 rail on the starting-zone walls.
- Cyclone dust collectors in the corners.
- Diffuse hall LED strips.

**Physics:** Earth gravity (the arena is at KSC).

**Viewport cameras:** at the guidebook's static and PTZ positions, plus an overhead view. Pick them from the Isaac viewport's camera menu.

Wall, duct and glass heights cross-check against Purdue Lunabotics' arena model (BSD-3).

**Digging.** The regolith is a deformable 2 cm heightmap, split into tiles so an edit re-meshes and re-collides only what it touches. Material slumps to BP-1's angle of repose (38°). Any node can edit it by publishing `[x, y, radius, depth]` on `/arena/dig` (depth > 0 digs, < 0 dumps). The sim publishes the true terrain (`/arena/heightmap`) and true volumes (`/arena/volume`: dug, dumped, and inside the berm box).

**Dig tool** ([sim/dig_tool.py](sim/dig_tool.py)). Two top-down panels:
- **Left, the true terrain,** with zones, obstacles, the rover and the D455's ground footprint, so you can see whether a dig is in view.
- **Right, SLAM's terrain map** placed in the arena frame. **E** cycles through:
  - `elevation`;
  - `change`: SLAM's own dug (blue) and dumped (red) map;
  - `error`: SLAM minus truth.

Controls (the dig tool window needs keyboard focus):
- **Driving, tank drive:** hold the keys; the rover stops when you let go or the window loses focus.
  - `W` / `S`: both tracks forward / back.
  - `A` / `D`: tracks in opposite directions, so the rover pivots in place.
  - `W+A` / `W+D`: one track stopped, so the rover arcs.
  - `1` `2` `3`: speed, 0.15 / 0.3 / 0.5 m/s (default 2).
- **Digging:** left-drag digs, right-drag dumps. The wheel or `[` `]` sets the brush size, `-` and `=` the depth per stroke.
- **Other keys:** `E` cycles the SLAM view, `R` re-aligns the SLAM map to the arena, `Q` quits and ends the run.

SLAM re-maps a dug hole or dumped pile once the D455 sees it: the cyan footprint in the left panel shows what the camera sees. The *change* view shows SLAM's own dug/dumped map, and RViz labels the new pits and piles.

The status bar shows true vs. SLAM dug and dumped volume, map error and camera drift. The SLAM map is placed using ground truth once (one instant of the camera's true pose). After that the panel shows only SLAM's own estimate, so drift and missed terrain changes appear as error.

**What the dig tool found and fixed** (the arena run, rover in the starting zone):
- **Height lock-in:** the terrain mapper's height-drift compensation could lock onto a burst of bad scans and keep the whole map about 1.5 m off. It now only follows slow drift (at most 5 cm per scan); sudden jumps are re-mapped by change detection instead. Map error went from 1.6 m to 0.3–0.8 cm median.
- **Gradual digging uncounted:** digging in shallow strokes didn't count as volume, because a cell's baseline kept tracking its height until its first "change" event. Baselines now freeze once a cell is well observed (σ < 1.2 cm). A 4 × 3 cm stroke dig went from about 50% to 81% of the true volume.
- **Hidden hole floors:** seen from one spot, the near part of a hole's floor is hidden behind its lip, so some volume only appears once the rover sees the hole from another side.
- **Tilt while driving:** SLAM's attitude is off by about 0.7–1.3° (up to ~3°) while driving, which smeared fan-shaped height errors into the map (up to ~20 cm at range). Each scan is now fitted to the map for a height offset *and* a small tilt (robust, rate-limited), removing those wedges in synthetic tests with ±2° of tilt error.
- **Odometry losses:** with a too-smooth regolith texture, stereo odometry lost track during turns. With footage-like clods, gravel and boot prints (image contrast σ 16 → 29) and a turn controller that doesn't overshoot, a lap ran with no losses and ≤ 9 cm position error.

`--eval` ([sim/arena_eval.py](sim/arena_eval.py)) automates this check and prints pose, map and volume errors as JSON.

Sim notes:
- **Turning:** pivot turns reach only about half the commanded rate in Earth gravity, even with the sim's gyro (yaw-rate) feedback loop. They no longer stall.
- **Camera locks:** robot and arena cameras are locked against viewport navigation. Orbiting while looking through the robot's D455 would otherwise move the sensor itself.
- **Speed:** about 0.37× real time on an 8 GB RTX 4060 laptop. Long runs are heavy: keep the laptop plugged in and cooled.

## Simulation (Isaac Sim 6)

```bash
./run_sim.sh                  # Isaac window + RViz + camera window, autonomous survey loop
./run_sim.sh --headless       # no Isaac window (saves GPU)
./run_sim.sh --dig-at 62      # dig a 0.25 m hole beside the loop just after lap 1 (sim time 62 s)
./run_sim.sh --teleop         # drive with the keyboard instead
./run_sim.sh --seed 3         # different terrain
```

[sim/isaac_lunar_sim.py](sim/isaac_lunar_sim.py) builds the scene from stock Isaac Sim assets and existing techniques:
- **Terrain:** procedural lunar terrain ([sim/lunar_terrain.py](sim/lunar_terrain.py), the OmniLRS approach). It combines fractal regolith, craters with raised rims, power-law rocks, a regolith texture and a low sun. The 4 m around the start is kept driveable.
- **Rover:** a Clearpath Jackal with skid-steer (tank) drive.
- **Camera:** Isaac's RealSense D455 model on a front mast, pitched 40° down so the rover never enters the view.
- **Physics:** lunar gravity (1.62 m/s²) and regolith friction.

It publishes the same topics as the real camera driver:
- `/camera/infra1|infra2` mono IR, with the 95 mm stereo baseline in `camera_info`.
- `/camera/depth` and `/camera/imu` (200 Hz, gravity included).
- Static camera TFs, plus `/clock` and `/ground_truth/odom`.

The same Lunabotics pipeline then runs with `sim:=true`. [sim/survey_driver.py](sim/survey_driver.py) drives a 2.5 m square twice, steering by ground truth; SLAM never sees it.

**One-time install** (about 40 GB download, about 25 GB on disk; needs a 595-branch NVIDIA driver):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv ~/isaacsim-env --python 3.12
VIRTUAL_ENV=~/isaacsim-env uv pip install "isaacsim[all,extscache]==6.0.1" \
    --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match
```

`run_sim.sh` accepts NVIDIA's Omniverse EULA (`OMNI_KIT_ACCEPT_EULA=YES`), so read it before your first run.

Notes:
- **Why 6.0.1:** Isaac Sim 5.1 crashes on startup with the 590/595 drivers, while 6.0.1 is validated on them.
- **Speed:** on an 8 GB RTX 4060 laptop, the sim runs at about 0.4× real time. Everything uses sim time, so SLAM is unaffected.
- **Texture:** the regolith texture has 1–4 cm clods and fine grain, the scale the camera actually resolves. With smooth noise alone, tracking dropped to about 40 inliers and was lost hundreds of times while moving. With clods it holds 120–160 inliers and loses nothing. Real lunar regolith is the hard case here, as the DEM-anchoring paper points out.
- **Rendering:** set up like a camera, not a game engine. DLSS, motion blur and temporally denoised lighting are off, and each frame gets 2 render subframes, because RTX's frame-to-frame accumulation smears moving images and breaks visual odometry.

## Tuning notes

- **Exposure:** IR auto-exposure picks 15–20 ms indoors with the projector off. That motion-blurs features and loses tracking. The firmware ignores `auto_exposure_limit`, so the config fixes exposure at 5 ms and raises gain to 64 to keep the same brightness. In a dim room, raise `depth_module.gain` (about 160). To go back to auto-exposure, set `depth_module.enable_auto_exposure: true`.
- **IR projector:** it's off on purpose. Its dot pattern moves with the camera, which corrupts feature tracking.
- **Timing:** IR and IMU timestamps both use librealsense `global_time`, meaning hardware time corrected to the host clock. Measured on this unit:
  - The stereo pair has identical timestamps on every frame.
  - The IMU runs at 200 Hz with 5.01 ms spacing.
  - No dropped or out-of-order samples.
  - Both streams drift the same ~0.2 ms/min relative to the host, so they stay in sync with each other.

  The startup warning `frame's time domain is HARDWARE_CLOCK` comes from the very first frames only, before the host-clock fit has enough samples.
- **Map database:** each mapping run starts a fresh map and overwrites `database_path`, so copy the `.db` somewhere to keep it. Appending sessions (`new_map:=false`) works, but it has crashed RTAB-Map once on startup (`Memory::addLink` assertion).
- **Odometry resets:** odometry re-initializes after 5 consecutive failed frames (`Odom/ResetCountdown`). Don't set it to 1. Each reset starts a new RTAB-Map session, and the displayed map jumps until a loop closure joins the sessions back together.
- **Startup:** keep the camera still for a second at startup. Odometry waits for the IMU to initialize gravity alignment.
- **Robot mounting:** publish a static TF `base_link -> camera_link` and set `frame_id: base_link` in the config's `stereo_odometry`, `rtabmap` and `rtabmap_viz` sections.
