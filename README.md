# lunabotics-d455-vislam

Stereo visual-inertial SLAM on an Intel RealSense D455, plus a Lunabotics terrain map with boulder and crater detection. It runs entirely on stock ROS 2 Humble packages, so nothing has to be compiled.

```bash
run/physical/lunabotics.sh                     # the physical D455: the full Lunabotics pipeline, same as the sim and arena runs
run/physical/lunabotics.sh localization:=true  # competition runs: localize in the surveyed map
run/physical/lunabotics.sh viz:=false          # headless, e.g. on the robot
```

`run/physical/lunabotics.sh` runs [Lunabotics terrain mode](#lunabotics-terrain-mode-semi-dynamic-map-for-digging):
- [fusion](#fusion-mode-most-accurate) tracking, i.e. visual-inertial odometry refined by ICP on dense depth;
- the terrain height map;
- [obstacle segmentation](#obstacle-segmentation-boulders-craters-walls-pillar): boulders, craters, walls and the pillar.

The basic stereo VISLAM pipeline below is still available as `run/physical/vislam.sh`:

| Stage | Package | What it does |
|---|---|---|
| Sensor | `realsense2_camera` | Publishes the left/right IR stereo pair (848x480 @ 30 Hz, hardware-synced, IR projector off, 5 ms fixed exposure) and the built-in BMI055 gyro + accel, merged into one `/camera/imu` topic at 200 Hz |
| IMU fusion | `imu_filter_madgwick` | Combines gyro and accel into an orientation estimate on `/imu/data` |
| Odometry | `rtabmap_odom/stereo_odometry` | Stereo visual odometry. It uses the IMU as the motion prior and for gravity alignment, and resets itself automatically if tracking is lost |
| SLAM | `rtabmap_slam/rtabmap` | Pose-graph SLAM with loop closure and IMU gravity constraints. It builds the dense map from the stereo pair (SGBM) and saves it to an SQLite database |
| Views | `rviz2` + `lunabotics_slam/scripts/vislam_viewer.py` | Opens two clean windows (described below) |

## Run scripts

Every mode has one script, grouped by where it runs. Run them from anywhere.

```
lunabotics_slam/              ROS 2 package: the SLAM stack (launch, config, rviz, nodes); see below
run/
├── physical/                 the real D455
│   ├── lunabotics.sh         full Lunabotics pipeline: fusion SLAM + terrain map + obstacles
│   ├── fusion.sh             fusion SLAM only: visual-inertial tracking refined by ICP on dense depth
│   ├── lidar.sh              depth as a simulated 3D LiDAR: ICP odometry + SLAM
│   ├── vislam.sh             basic stereo visual-inertial SLAM
│   └── reconstruct.sh        colour 3D scan, exported as a textured mesh
├── sim/                      Isaac Sim 6
│   ├── procedural/
│   │   └── lunar_terrain.sh  procedural lunar terrain: survey loop, teleop, dig test
│   └── arena/
│       ├── arena.sh          Artemis Arena replica: dig tool, survey, ground-truth evaluation
│       └── cosmos.sh         NVIDIA Cosmos 3 Edge makes the decisions of a full 15-min run, on SLAM
└── export_mesh.sh            any saved map (physical or sim) -> textured mesh + point cloud
```

## ROS 2 package for the robot (`lunabotics_slam/`)

The SLAM stack itself lives in a ROS 2 package: launch files, configs, RViz layouts, the four Python nodes and the RealSense udev rules. The run scripts above use it straight from this folder. On the robot, put the repo in a colcon workspace and run the arena pipeline as an installed package, with the camera mounted on your robot's body:

```bash
cd ~/ros2_ws && colcon build --packages-select lunabotics_slam && source install/setup.bash
ros2 launch lunabotics_slam lunabotics.launch.py base_frame:=base_link camera_mount:="0.30 0 0.42 0 0.70 0" viz:=false
```

`base_frame` makes odometry and SLAM report your robot frame. `camera_mount` (`"x y z roll pitch yaw"` of `camera_link` in it) publishes the static TF, and `odom_guess_frame` adds wheel odometry as the motion guess. Install steps, all arguments and topics are in [lunabotics_slam/README.md](lunabotics_slam/README.md).

## Setup (once per machine)

```bash
sudo ./setup.sh   # installs any missing apt packages + RealSense udev rules
# then unplug and replug the D455
```

The udev rules are required for the IMU. Without them, the camera's HID/IIO sensor nodes stay root-only. The driver then fails with `Failed to open scan_element` / `Hid device is busy` and keeps resetting the USB device.

## Basic stereo VISLAM (`run/physical/vislam.sh`)

```bash
run/physical/vislam.sh                                # new map with both windows (saved to ~/.ros/d455_vislam.db)
run/physical/vislam.sh viz:=false                     # headless, e.g. on the robot
run/physical/vislam.sh new_map:=false                 # extend the saved map instead of starting over
run/physical/vislam.sh localization:=true             # localize in the saved map, no new mapping
run/physical/vislam.sh database_path:=$HOME/arena.db  # use a specific map file
run/physical/vislam.sh dynamic_map:=true              # map forgets surfaces that are gone (e.g. dug terrain)
run/physical/vislam.sh config:=lunabotics_slam/config/my_room.yaml    # use a different settings file
run/physical/vislam.sh rtabmap_viz:=true              # also open the stock RTAB-Map GUI
```

### Settings

All camera, IMU, odometry and SLAM settings live in [lunabotics_slam/config/d455_vislam.yaml](lunabotics_slam/config/d455_vislam.yaml), one section per node, each setting commented. Edit it and rerun; nothing needs rebuilding. For a variant, such as a dim room with `depth_module.gain: 160`, copy the file and pass it with `config:=`. The launch flags above only pick modes.

`dynamic_map:=true` loads [lunabotics_slam/config/dynamic_map.yaml](lunabotics_slam/config/dynamic_map.yaml) on top of the main settings. That file turns on ray tracing in RTAB-Map's 3D voxel map (OctoMap), so voxels the camera can now see through, such as dug-out ground, are removed. RViz then shows `/octomap_occupied_space` instead of `/cloud_map`.

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
run/physical/lidar.sh                     # map with RViz (saved to ~/.ros/d455_lidar.db)
run/physical/lidar.sh viz:=false          # headless
run/physical/lidar.sh localization:=true  # localize in the saved LiDAR map
```

This mode treats the D455 like a 3D LiDAR. Its depth stream is converted into a point cloud on `/lidar/points`: about 8,500 points per scan at 30 Hz, 5 cm spacing, 0.3–6 m range. Mapping then runs RTAB-Map's own 3D-LiDAR pipeline (`rtabmap_examples/lidar3d.launch.py`): ICP scan-to-map odometry with the IMU as the rotation prior, then scan-based SLAM. No visual features are used, so the IR projector is on for dense depth. RViz shows the voxel map coloured by height, the live scan in white, the trajectory and the camera frustum.

To swap in a real LiDAR later, publish its cloud on `/lidar/points` and drop the camera and `point_cloud_xyz` nodes. Settings live in [lunabotics_slam/config/d455_lidar.yaml](lunabotics_slam/config/d455_lidar.yaml).

ICP needs geometry to lock onto. A flat floor plus one wall, or a scene with nothing within 6 m, can make it slide. The visual mode (`run/physical/vislam.sh`) is the more robust tracker; this mode gives the denser, LiDAR-style map.

## Fusion mode (most accurate)

```bash
run/physical/fusion.sh                     # map with both windows (saved to ~/.ros/d455_fusion.db)
run/physical/fusion.sh viz:=false          # headless
run/physical/fusion.sh localization:=true  # localize in the saved fusion map
```

This mode combines the strengths of the other two using one D455:

1. The IR projector switches on every other frame (`depth_module.emitter_on_off`). [lunabotics_slam/scripts/emitter_splitter.py](lunabotics_slam/scripts/emitter_splitter.py) reads each frame's `frame_emitter_mode` metadata and splits the stream by projector state:
   - Projector-off IR pairs go to `/split/infra*`. They feed stereo visual-inertial odometry, the robust tracker from `run/physical/vislam.sh`, at 15 Hz.
   - Projector-on depth goes to `/split/depth`. It becomes a dense point cloud on `/lidar/points`.
2. RTAB-Map uses the visual odometry as the motion guess. ICP on the dense clouds then refines every new keyframe link (`RGBD/NeighborLinkRefining`), and adds local loop closures by matching nearby scans (`RGBD/ProximityBySpace`). Visual loop closures are refined with ICP too (`Reg/Strategy 2`). The map is built from the dense clouds.
3. Where a scan is too flat to constrain every direction (a wall plus floor), ICP corrects only the constrained axes and keeps the visual estimate for the rest (`Icp/PointToPlaneLowComplexityStrategy`). That's RTAB-Map's answer to ICP degeneracy, and it covers the weakness of the LiDAR mode.

Measured on this D455:
- 15 Hz on each half of the split.
- Every ICP refinement was accepted, with an average inlier ratio of 0.34.
- About 0.3 s per map update, even with a keyframe on every update.

Settings are in [lunabotics_slam/config/d455_fusion.yaml](lunabotics_slam/config/d455_fusion.yaml). The next accuracy step is a wheel odometry + IMU guess, from `robot_localization` on the robot.

## Lunabotics terrain mode (semi-dynamic map for digging)

```bash
run/physical/lunabotics.sh                     # survey: map the arena + record the terrain baseline
run/physical/lunabotics.sh localization:=true  # runs: localize in that map, load the terrain, report dug volume
```

This mode runs fusion localization (above) plus [lunabotics_slam/scripts/terrain_mapper.py](lunabotics_slam/scripts/terrain_mapper.py). The mapper keeps a 2.5D height map (5 cm cells) that **re-maps areas that change**, such as a hole being dug or a berm being built. Each cell is updated with these existing techniques:
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

### Obstacle segmentation (boulders, craters, walls, pillar)

[lunabotics_slam/scripts/obstacle_segmenter.py](lunabotics_slam/scripts/obstacle_segmenter.py) runs twice a second. It shows the driver where the obstacles are and gives a planner an occupancy grid. It uses only OpenCV and SciPy (connected components, contours, `minAreaRect`, image filters), which the pipeline already needs.

**Boulders and craters,** from the terrain map:
1. **Local ground:** a 1 m window around each cell, taken as a low percentile and then a median with bumps and pits removed. Slopes, ducts and crater rims don't shift it.
2. **Threshold:** cells more than 8 cm above the local ground seed a bump; cells more than 6 cm below it seed a pit. A morphological open removes single-cell noise.
3. **Outline:** each seed grows out through connected cells to a shallow edge threshold (3 cm for bumps, 1.5 cm for pits; at most 15 cm), and unseen cells inside are filled. Without this, a bowl-shaped crater was sized by its deep middle only, about 60% of its real width.
4. **Rim:** a crater is then widened out to the crest of its raised rim, so the outline covers everything a wheel should keep off. The crest is the ring around the crater (searched up to 25 cm out) with the highest median height; if it isn't seen, the crater is widened by 8 cm.
5. **Blobs:** connected blobs are sized and labelled:

| Label | RViz colour | Rule |
|---|---|---|
| boulder | orange | bump 12–90 cm across |
| crater | blue | pit 15–110 cm across (size given out to the rim crest) |
| dug pit | cyan | pit the robot dug (terrain change < −4 cm since the baseline) |
| pile | yellow | bump the robot dumped (change > +4 cm), e.g. the berm |
| structure | (not drawn) | long, large or taller than 60 cm; kept out of the boulders, drawn as wall / pillar below |

**Walls and the pillar,** from RTAB-Map's 3D map (`/cloud_map`). The terrain map holds one height per cell, so it averages vertical faces into a blur; the 3D map keeps them.
1. **Ground:** the lowest map point per cell, through the same local-ground filter. Taking it from the same cloud means drift between the two maps doesn't matter.
2. **Tall cells:** cells with at least 2 map points more than 12 cm above that ground.
3. **Blobs:** connected tall blobs are sized with `cv2.minAreaRect` and labelled:

| Label | RViz colour | Rule |
|---|---|---|
| wall | light grey | longer than 1 m: bin walls with their ducts |
| pillar | magenta | compact (20–90 cm) and higher than 35 cm; pieces less than one pillar apart are merged |

Terrain bumps that touch a wall or the pillar (duct pieces, the column's foot) are dropped from the boulders.

The D455 is pitched down, so it sees things more than 35 cm high only within about 1.3 m. Until the rover passes the column that close, the column shows as a large bump, not a pillar.

Outputs:
- `/terrain/obstacles`: MarkerArray. Each obstacle is drawn as its segmented cells:
  - boulders and craters as tiles draped on the terrain;
  - walls and the pillar as columns up to their seen height.

  Each also gets an outline and a label such as "crater 41 cm". It is shown in RViz as *Obstacles*.
- `/terrain/obstacle_grid`: OccupancyGrid. Boulders, craters, walls and the pillar are 100, other known cells 0, unseen cells −1. It is shown in RViz as *Obstacle grid* (off by default).

Offline test on 5 simulated arena rounds, with 1 cm map noise:
- found 30 of 31 boulders and 15 of 16 craters, with no false positives;
- sizes: boulders 95% of true diameter (87–98%);
- craters: 100% of the true hole covered (median), 94% of the hole out to the rim crest; outlines are 1.15× the hole's diameter;
- median centre error 1.0 cm;
- about 110 ms per update; walls and the pillar about 0.3 s per new 3D map, at 400k points.

Walls and the pillar, synthetic 3D points on the same 5 rounds:
- wall ring found every time;
- pillar found every time once seen more than 35 cm up, centre within 14 cm;
- no boulder labelled a pillar.

An obstacle has to be seen before it can be segmented. The camera looks about 0.5–3.5 m ahead.

Settings are in the `obstacle_segmenter` section of [lunabotics_slam/config/lunabotics_terrain.yaml](lunabotics_slam/config/lunabotics_terrain.yaml).

### Terrain settings

Settings are in [lunabotics_slam/config/lunabotics_terrain.yaml](lunabotics_slam/config/lunabotics_terrain.yaml). The two-step workflow is the arena version of anchoring to a prior map, which is how the [DEM-anchored lunar SLAM paper](https://arxiv.org/abs/2603.17229) limits drift.

## 3D reconstruction (`run/physical/reconstruct.sh`)

```bash
run/physical/reconstruct.sh               # scan in colour; Ctrl+C stops and exports the mesh
run/physical/reconstruct.sh viz:=false    # headless
run/physical/reconstruct.sh --no-export   # scan only (saved to ~/.ros/d455_reconstruct.db)
run/export_mesh.sh                        # export the last scan again
run/export_mesh.sh ~/.ros/lunabotics.db   # export any other map, physical or sim
```

This mode turns on the D455 colour camera and is built only for producing a 3D model:
1. The camera publishes colour plus depth aligned to it, with the IR projector always on. The colour camera's IR-cut filter hides the dots, so they never reach tracking.
2. `rgbd_odometry` tracks on colour + depth, with the IMU as the motion prior. `rtabmap` keeps the colour image and depth of every keyframe (2 Hz, every 5 cm or 0.05 rad) and closes loops. RViz shows the map in its real colours.
3. On exit, [run/export_mesh.sh](run/export_mesh.sh) runs RTAB-Map's stock `rtabmap-export` on the saved database. It re-assembles the depth of every keyframe on the optimized poses, meshes it, and textures it from the keyframe images.

Output goes to `~/.ros/reconstructions/<map name>/`: `<name>_mesh.obj` (with `.mtl` and a `.jpg` texture atlas) and `<name>_cloud.ply`. Open them in MeshLab, Blender or CloudCompare. Extra args go to `rtabmap-export`, e.g. `--max_range 3` or `--poisson_size 0.02`; `rtabmap-export --help` lists them all.

`run/export_mesh.sh` works on every map this repo saves, not just reconstruction scans. Maps from the other modes have no colour, so their meshes are textured with the greyscale IR image. Exporting each existing map took a few seconds.

Scanning tips:
- Move slowly and revisit areas from a few angles. Every keyframe is a texture source, and overlapping views fill holes.
- D455 depth error grows to about 2% of range at 4 m, so the map ignores anything farther than that (`Grid/RangeMax`).
- Lighting matters: colour auto-exposure is capped at one frame time to keep 30 Hz, so dim rooms come out dark rather than blurred.

Settings are in [lunabotics_slam/config/d455_reconstruct.yaml](lunabotics_slam/config/d455_reconstruct.yaml).

## Lunabotics arena simulation + dig tool

```bash
run/sim/arena/arena.sh                    # arena + SLAM + dig tool; drive with W A S D in the dig tool window
run/sim/arena/arena.sh --survey           # the rover drives survey loops on its own
run/sim/arena/arena.sh --seed 4           # another round: new boulders, craters and start pose
run/sim/arena/arena.sh --start-heading N  # start facing N/E/S/W (default: random, like the direction wheel)
run/sim/arena/arena.sh --eval --headless  # check SLAM vs. ground truth: survey lap, dig + dump, second lap, report
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

## Cosmos 3 Edge arena run (`run/sim/arena/cosmos.sh`)

```bash
sim/setup_cosmos.sh                          # once: Python env + model (~12 GB download)
run/sim/arena/cosmos.sh                      # full 15-min run: Isaac (arena from above), RViz (SLAM map), model window
run/sim/arena/cosmos.sh --headless           # no Isaac window and no RViz
run/sim/arena/cosmos.sh --seed 4             # another round: new boulders/craters and start pose
run/sim/arena/cosmos.sh --minutes 5          # shorter run
run/sim/arena/cosmos.sh --gt-pose            # no SLAM: true pose, depth-camera hazards only
run/sim/arena/cosmos.sh --think-tokens 128   # reason on every step (default: only after trouble)
run/sim/arena/cosmos.sh --quant bf16         # model precision: int8 (default), int4, bf16, nf4
```

NVIDIA's [Cosmos 3 Edge](https://huggingface.co/nvidia/Cosmos3-Edge) vision-language model is the decision maker of a whole competition run, on top of the arena SLAM stack. The stack is the same as `arena.sh`: fusion SLAM, the terrain map and obstacle segmentation (`lunabotics_slam/launch/lunabotics.launch.py sim:=true`). Each step the model sees the rover's camera and a status report built from SLAM, then picks one action.

| Piece | File | What it does |
|---|---|---|
| SLAM | [lunabotics_slam/launch/lunabotics.launch.py](lunabotics_slam/launch/lunabotics.launch.py) | Pose, terrain map (measured berm volume) and segmented obstacles (`/terrain/obstacles`, `/terrain/obstacle_grid`). |
| Model | [sim/cosmos_server.py](sim/cosmos_server.py) | Cosmos 3 Edge quantized to int8 with `torchao` and compiled, served on `http://127.0.0.1:8000/v1` (OpenAI chat API, the same as the model card's vLLM server). |
| Agent | [sim/cosmos_agent.py](sim/cosmos_agent.py) | Builds the status report, asks the model, executes the action under the arena rules, prints the score card. |
| Viewer | [sim/cosmos_viewer.py](sim/cosmos_viewer.py) | Live window: the camera frame the model sees, its status report, its reasoning as it streams, and the chosen action. |
| Benchmark | [sim/cosmos_bench.py](sim/cosmos_bench.py) | 20 arena situations with known good actions: decision quality and speed for any server setting. |
| Run | [run/sim/arena/cosmos.sh](run/sim/arena/cosmos.sh) | Starts the model, the arena, SLAM, the viewer and the agent. |

**What the model gets each step:**
- **Camera:** the D455 IR image (grayscale, 512 × 320).
- **SLAM pose** and its zone.
- **SLAM obstacle map:** the nearest obstacles the segmenter found ("boulder 34 cm at (4.10, 1.00): 1.2 m, +15 deg").
- **Berm volume** in the target box, as measured by the SLAM terrain map.
- **Depth-camera hazards** per sector (left / ahead / right): points more than 10 cm above or 8 cm below the local ground.
- **Bucket state** with the consequence spelled out: where a dump right now would land (inside the box or not), or whether digging is possible here.
- **Time left, cycles, penalties,** the last few actions and their results.
- **A suggested next action, placed last.**

**What it can do:**
- **`go_to X Y [H]`** plans an A* path on the SLAM obstacle grid (inflated by 0.32 m), follows it, replans every 2 s on the newest map, and optionally ends facing heading `H`.
- **`turn DEG`, `drive M`, `dig`, `dump`** and **`wait S`**.
- **Safety:** a move stops at a depth-camera hazard within 0.75 m in the rover's path or at a wall, and the model is told why.
- **Hazard memory:** the hazards that stopped a move go into the planning grid for 2 min, so the next `go_to` routes around them even before SLAM maps them. Without this, the rover kept planning straight into a crater the depth camera saw but the SLAM grid didn't have yet.
- **Planner recovery:** if the map boxes the rover in, `go_to` retries like a costmap-clearing recovery. It first ignores SLAM obstacle cells within 0.8 m of the rover, then uses only walls and seen hazards; the model is told it's a recovery. A path is only replanned when the newest map blocks it, because replanning on every map update made the rover chase a new first waypoint each time and re-pivot instead of driving.
- **Own dig site:** the rover digs right in front of itself, so its own shallow scoop is excluded from both the depth hazards and the planning grid. It's drivable; the dip threshold is meant for 12–22 cm craters.
- **Tracking loss:** while SLAM's visual odometry reports lost (null `/odom`), the rover stops and the current move's timer is held, so it never steers on a frozen pose. Short losses (about 0.1–0.2 s) happen when pivoting next to a bare wall.

**Localization:** the SLAM map frame is tied to the arena frame once, from one ground-truth camera pose at the start. This stands in for localizing on the start-zone fiducials, which the rules allow. From then on the robot knows its pose only from SLAM. The walls and the column are not given to the planner as map features (guidebook §16), so the column has to be mapped by SLAM like everything else. The judges' side (penalties, the score card's berm volume) uses ground truth.

**Digging and dumping** use the sim's terrain editing (`/arena/dig`); no parts move.
- **`dig`** (20 s) scoops a shallow patch in front of the rover: six strokes over about 0.5 × 0.75 m, at most 3 cm deep. It fills the bucket with 10,000 cm³, and is refused unless every stroke lands inside the excavation zone and clear of the walls/ducts. (An earlier four-strokes-in-a-line dig made a 12 cm trench that the rover then bogged down in.)
- **`dump`** (10 s) drops the load as a pile 0.65 m in front of the rover, which then slumps to BP-1's angle of repose. It is refused if the pile would land outside the construction zone. The suggested dump spots fill the box far to near (x = 6.1, 5.6, 5.1, 4.7), so the rover never has to stand on an earlier pile; after that, loads stack on the nearest pile.

**Rules and scoring** (2027 guidebook, sec. 11–17):
- **Run:** 15 minutes, random start position and heading.
- **Digging** only in the excavation and starting zones.
- **Berm:** only volume inside the 2.2 × 0.9 m box at (5.38, 0.9) counts.
- **Obstacle penalty:** −50 for the first boulder contact or crater crossing in the obstacle zone, −25 after, at most −100.
- **Autonomy:** 1400 points with ≥ 2 complete dig → dump cycles, scaled by min(1, berm / 25,000 cm³).
- **Not simulated:** BCP mass/energy, bandwidth, dust.
- **Score card:** also reports SLAM's pose error against the truth.

**Output:** every step (camera frame, status, reasoning, action, true pose) is saved to `~/.ros/cosmos_runs/<time>/` (`step_NNN.jpg`, `log.jsonl`, `score.txt`).

**Inference speed.** Measured with `sim/cosmos_bench.py` on the 8 GB RTX 4060 laptop: 20 situations × 2–3 samples, temperature 0.3. There is about ±7% sampling noise at n = 40–60.

| Setup | Good decisions | Seconds per decision |
|---|---|---|
| Start: bitsandbytes 4-bit, 384 reasoning tokens | 65% | 6.4 |
| bf16 (official precision), 384 reasoning tokens | 72% | 7.8 |
| int4 + compile, 384 / 192 / 64 reasoning tokens | 75% / ~84% / 90% | 6.2 / 2.7 / 1.5 |
| bf16, answer directly, 128 tokens after trouble | 83% | 0.95 |
| int4 + compile, answer directly, 128 after trouble | 93% | 1.00 |
| **int8 + compile, answer directly, 128 after trouble (default)** | **88%** | **0.73 (median 0.44)** |

What made it faster:
- **Quantization with fast kernels:** decoding is memory-bound. On this GPU bf16 already runs at its bandwidth limit (57 tok/s), so `torchao` int8/int4 weight-only + `torch.compile` with a static KV cache reach 95–112 tok/s. int4 quality matched bf16 at the same budget. The community bitsandbytes 4-bit build was both slower and worse.
- **Static cache size:** every compiled decode step attends over the whole cache, so a 3,072-slot cache ran at 78 tok/s and a 1,536-slot one at 112. The server keeps two sizes (1,792 and 2,560) and uses the smallest that fits; prompts are ~1,100–1,450 tokens.
- **Less reasoning:** with the status report spelling out consequences ("a dump right now would land … INSIDE the berm box"), reasoning didn't improve routine decisions; long reasoning made them worse, as the small model second-guesses itself. The agent answers directly and only reasons (128 tokens, then forced to `ACTION:`) after a refused, stopped or timed-out action or a penalty.
- **Prefill matters too:** with short answers, int8's fast prefill (~0.2 s, against int4's ~0.55 s) makes it the quickest overall.

**Known issue: terrain-map phantoms.** In some runs, right after the first dump, the SLAM terrain map grew phantom piles and craters around the rover, and its berm measurement jumped (705,616 cm³ "measured" against 9,958 true). The likely cause is that the fresh 10–18 cm pile right in front of the camera falls inside the terrain mapper's 15 cm drift window, so it biases the scan-to-map height/tilt correction. That's the same family as the earlier "height lock-in" bug. The planner recovery above keeps the rover moving through it, but it needs a fix in `lunabotics_slam/scripts/terrain_mapper.py`.

**Measured run** (seed 0, SLAM, 3 min, before the speed work): 1 complete cycle, 9,955 cm³ in the box (SLAM measured 8,759 cm³), no penalties, SLAM pose error 5 cm mean / 10 cm max.

## Simulation (Isaac Sim 6)

```bash
run/sim/procedural/lunar_terrain.sh              # Isaac window + RViz + camera window, autonomous survey loop
run/sim/procedural/lunar_terrain.sh --headless   # no Isaac window (saves GPU)
run/sim/procedural/lunar_terrain.sh --dig-at 62  # dig a 0.25 m hole beside the loop just after lap 1 (sim time 62 s)
run/sim/procedural/lunar_terrain.sh --teleop     # drive with the keyboard instead
run/sim/procedural/lunar_terrain.sh --seed 3     # different terrain
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

`run/sim/procedural/lunar_terrain.sh` (like `run/sim/arena/arena.sh`) accepts NVIDIA's Omniverse EULA (`OMNI_KIT_ACCEPT_EULA=YES`), so read it before your first run.

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
