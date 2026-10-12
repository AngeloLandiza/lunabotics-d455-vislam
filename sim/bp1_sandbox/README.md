# BP-1 sensor sandbox (step 2, v0.2)

A plain-Python, CPU-only stand-in for the arena: it builds random 2027-style arenas, simulates a LiDAR and the D455 over them with noise and dust, and scores boulder and crater detectors against ground truth. It is the first layer of the BP-1 simulation; the same sensor models are meant to sit on top of Isaac Sim's ideal ray hits later (see "Path to Isaac Sim").

## What it models

| Part | Model | Evidence |
| --- | --- | --- |
| Arena | 6.8 m x 5.0 m height field at 1 cm, walls, a central column, rough raked surface, ~2 cm gravel, ≥3 boulders (30-40 cm wide), ≥3 craters (≤ ~40-50 cm) | Sizes Verified from the 2027 guidebook; zone layout, column size, roughness and heights Assumed |
| LiDAR | Ray marching against the height field; return power ∝ ρ·cos(incidence)/r²·exp(−2τ); blind zone; Gaussian range noise | Power law and two-way attenuation from Hahner et al., arXiv 2108.05249 (Eq. 17); BP-1 reflectivity Assumed (0.10) |
| LiDAR scan patterns | Generic 16-beam spinning; approximate non-repetitive rosette with 70.4° FOV and 0.21 m blind zone | Rosette FOV and blind zone are the MID-70 values in Xie et al. 2024; the curve shape is Assumed |
| Dust | Gaussian plume of extinction α(x,y,z); two-way attenuation of true returns; discrete particle echoes kept in first-return mode | Model form from Hahner et al. (fog, arXiv 2108.05249; snowfall, arXiv 2203.15118); every dust number Assumed |
| D455 | Per-pixel ray cast at 212x120, 87°x58° FOV; σz = z²·σd/(f·B), f = 674 px, B = 95 mm, σd = 0.32 px; invalid pixels rise as transmittance falls | FOV and baseline Verified (Intel); σd Inferred from "<2% at 4 m"; dust effect Assumed |
| Detectors | baseline: fixed height thresholds per 5 cm cell. carving: thresholds scale with 3σ of range noise, free-space carving bounds hidden crater floors, and a solidity check drops high returns that another ray passed beneath. log-odds: each return and each distinct ray adds evidence per cell, frame by frame (+0.85 support, -0.4 contradiction, clamped to [-2, 3.5]); rays passing below the ground plane count as crater evidence; an object is accepted when the evidence summed over its cells passes a per-sensor threshold | carving and log-odds are our own combinations (Inferred) of CFAR-style normalisation, occupancy carving, Pairet et al.'s log-odds steps and Hines et al.'s virtual surfaces (research/step2/18); the log-odds step sizes are Pairet et al.'s, the thresholds were tuned on seeds 1000-1015 |

Since 2026-10-12 the robot body, its self-occlusion and wheel ruts are modelled too (section "Team robot, Unitree L1, self-mask and ruts"). Not modelled yet: stereo matching on texture (depth is ray-cast plus noise), dust seen by the D455 as false depth, multiple scattering, sun glare, slip, and terrain deformation from digging.

## Run it

```
python3 -m pip install numpy scipy matplotlib
python3 evaluate.py 60 results_v02 2000   # Monte Carlo on seeds 2000-2059, about 35 s per trial per core
python3 figures.py results_v02            # figures + tables.md
python3 tests/test_bridge.py       # Isaac bridge checks (needs: python3 -m pip install usd-core)
python3 isaac_bridge.py 1          # usd/arena_seed1.usda + truth.json for Isaac Sim
./run_isaac_capture.sh             # Isaac Sim 6.0.1 headless capture of that arena + isaac_validate.py (see below)
```

## Files

- `params.py`: every number with its status (Verified / Inferred / Assumed) and source. Change values here only.
- `arena.py`, `raycast.py`, `sensors.py`, `dust.py`: the world and sensor models.
- `detect.py`: grid accumulation, ground fit, both detectors, and scoring.
- `evaluate.py`, `figures.py`: the experiment and its plots. `results/` is v0 (baseline and carving, seeds 1000-1059); `results_v02/` adds the log-odds detector on fresh seeds 2000-2059 that no threshold was tuned on. Result folders are not committed; rerun the two commands above to rebuild them.
- `isaac_bridge.py`: exports an arena to USD for Isaac Sim, and applies the BP-1 layer to ideal LiDAR ranges or depth images from any source.
- `tests/test_bridge.py`: checks that the BP-1 layer gives identical output from our ray marcher and from outside hits, and that the USD mesh matches the height field.
- `isaac_capture.py`, `isaac_validate.py`, `run_isaac_capture.sh`: the Isaac-side capture and its check (section "Isaac Sim capture"). Output goes to `isaac_out/`, which is not committed.
- `approach.py`, `speed_gate.py`: frame-by-frame approach runs and their scoring for confirmation thresholds, speeds and a speed gate (section "Approach runs and speed"). Output goes to `results_approach*/`, which is not committed.
- `isaac_approach.py`, `run_isaac_approach.sh`: approach runs on Isaac Sim hits next to the sandbox (section "Approach runs on Isaac Sim hits"). `isaac_selfmask.py`, `run_isaac_selfmask.sh`: the robot's self-occlusion in Isaac Sim. Small results go to `results_isaac/`.
- `live_view.py`, `live_view.rviz`, `run_live.sh`: live RViz2 view of one approach run (section "Live view").
- `robustness.py`: floor roughness, floor waves, attitude error and the local ground fit, scored at the speeds that passed (section "Floor and tilt cases"). Output goes to `results_rob/`, which is not committed.
- `robot.py`, `selfmask.py`, `ruts.py`, `selfmask_eval.py`, `run_selfmask.sh`: the team robot from its URDF, the L1 self-mask, wheel ruts and their 48-arena cases (section "Team robot, Unitree L1, self-mask and ruts"). Output goes to `results_selfmask/`, which is not committed; `summaries/` keeps the small result files.
- `refs/guidebook2027/`: guidebook 2027 Figure 7 (BP-1 laser reflectivity photos) and `measure_fig7.py`, which reads BP-1's laser-line brightness against the target patches.
- `telemetry.py`: progress lines for the dashboard in `sim/dashboard/` (`python3 sim/dashboard/server.py`, then http://127.0.0.1:8765/).
- `usd/arena_seed1.usda`: one exported arena (Z up, metres, origin at the start-zone corner) with its obstacle list in `arena_seed1.usda.truth.json`.

## Path to Isaac Sim

NVIDIA lists an x86_64 machine with an RTX 4080 (16 GB VRAM) as Isaac Sim's minimum, and Jetson is not on its requirements page; the capture below runs on an 8 GB RTX 4060 Laptop GPU anyway. The pages read in step 2 document no Livox-style pattern, no LiDAR range noise and no D455 stereo noise, and one forum reply says fog does not affect RTX LiDAR in Isaac Sim 4.2.0 (research/step2/16 and 17, with corrections in research/step2/verify). So the plan is a hybrid (Inferred):

1. `isaac_bridge.py` exports the same random arena to USD (`export_arena_usd`), with a static collider and the true obstacle list.
2. Isaac Sim renders that arena and gives ideal LiDAR ranges and ideal depth images.
3. `lidar_from_ideal` and `d455_from_ideal` add BP-1 reflectivity, dust and noise with the same code the sandbox uses (`Lidar.apply`, `D455Depth.apply`); the test proves the two paths match exactly.
4. `detect.py` and `score` run unchanged on the result.

Done since: the Isaac-side capture below. Still not done: a dust plume that follows the digging tool, and loading the USD collider with a driving robot.

## Isaac Sim capture

`run_isaac_capture.sh` runs `isaac_capture.py` with `~/isaacsim-env/bin/python` (Isaac Sim 6.0.1, pip install, headless), then `isaac_validate.py` with the system `python3`. Tested on an RTX 4060 Laptop GPU (8 GB): peak 1.66 GB of VRAM by nvidia-smi, about 16 s for 8 poses.

- **Arena**: `usd/arena_seed1.usda`, opened as-is (Z up, metres).
- **Poses**: robot `(x, y, yaw)` list (`--poses`); both sensors placed with `sensors.sensor_pose()` and `params.MOUNTS`.
- **LiDAR**: an RTX `OmniLidar` with a custom rotary pattern equal to `spin16`: 16 emitters at -15 to +15 deg, 900 azimuth ticks (0.4 deg), 8 m far range, one return, `instantLidar`. Built-in noise is off: `rangeAccuracyM` (schema default 0.02 m), azimuth, elevation and origin errors set to 0. Isaac returns azimuth, elevation and range per ray; the script stores them in `spinning_dirs` order.
- **RTX dropout**: each RTX scan drops about 1.5-3% of its returns at random, and no documented attribute turns this off. The script merges 4 scans per pose (`--scans`); returned ranges agree between scans to 0.0 cm.
- **Depth**: a `UsdGeom.Camera` with the `distance_to_image_plane` annotator, as in `sim/isaac_lunar_sim.py`. Isaac renders square pixels only, so 87 x 58 deg at 212 x 120 cannot be rendered directly. The script renders 1060 x 622 (`--upsample 5`) and resamples to the sandbox pixel grid: columns are exact, rows are interpolated.
- **Output**: `isaac_out/capture.npz`. Per pose it holds the origin, rotation, merged ranges, Isaac ray directions, the per-scan ranges, and the depth image (rows top to bottom), ready for `isaac_bridge.lidar_from_ideal` / `d455_from_ideal`.

Agreement with the sandbox (seed-1 arena, 8 poses, run 2026-10-11):

| Comparison | median abs. diff | 95th pct | one side hits only |
| --- | --- | --- | --- |
| LiDAR vs sandbox ray march | 0.04-0.19 cm | 1.04-1.22 cm | 0.04-0.15 % |
| LiDAR vs exact USD mesh | 0.00 cm | 0.07-0.10 cm | 0.01-0.10 % |
| Depth vs sandbox ray march | 0.00 cm | 0.02-0.14 cm | 0 % |
| Depth vs exact USD mesh | 0.00 cm | 0.00 cm | 0 % |

The LiDAR's 1 cm 95th percentile against the sandbox comes from the export: the USD mesh samples the 1 cm height map every 2 cm. Against the mesh itself, Isaac is exact to 1 mm. A single RTX scan without merging misses 1.3-3.0 % of rays.

Detection (logodds, two 4-frame approaches, clear air and a plume 0.8 m ahead): over 16 cases per source (2 approaches x LiDAR and D455 x clear and dust x 1 and 4 frames), the sandbox found 69 obstacle detections with 2 false alarms, and Isaac + BP-1 layer found 68 with 3. Per-case results are in `isaac_out/validation.json`. The differences are single obstacles near the threshold (Inferred: the same noise draws land on slightly different ideal ranges).

## Live view (RViz2)

`./run_live.sh [flags]` starts rviz2 with `live_view.rviz` and runs `live_view.py`. That script replays `approach.trial` for one arena seed, one sensor, and clear air or dust, frame by frame. It needs ROS 2 Humble (rclpy, rviz2) and the system `python3`.

```
./run_live.sh                                   # seed 1, spin16, clear air, 2 frames/s
./run_live.sh --sensor d455 --dust --loop       # D455 in the dust plume, replaying the run
./run_live.sh --l-dec-lidar 4.0 --paused        # another confirmation threshold; start paused
```

- **Flags**: `--seed`, `--sensor spin16|d455`, `--dust`, `--rate` (frames/s), `--l-dec-lidar` / `--l-dec-d455` (confirmation thresholds, defaults from params.py: 5.1 and 0.85), `--paused`, `--loop`.
- **Keys** in the terminal: space pause/resume, `n` step, `r` restart, `q` quit.
- **Speed**: the first pass shows frames as they are computed. A spin16 frame in dust takes about 1 s on the laptop, so playback can be slower than `--rate`. With `--loop`, later passes replay the cached frames at `--rate`.
- **What is shown** (frame `map` = arena frame):
  - terrain coloured by height, and the true obstacles as translucent labelled cylinders (boulder orange, crater blue, column grey);
  - the robot footprint (TF `base_link`), its corridor (half-width 0.40 m, from the front edge 0.35 m ahead of centre), and the path driven so far;
  - the current scan, with surface returns in green and dust echoes in orange, plus the dust plume as a +-2 sigma ellipsoid;
  - the log-odds layers `lp` (blue to red, full cells) and `ln` (teal to magenta, smaller cells);
  - every candidate cluster, labelled with its kind, pooled evidence, state and truth class. Red is confirmed (evidence >= threshold), yellow is unresolved (0.85 <= evidence < threshold), grey is faint;
  - the speed gate: 1.0 m/s, or 0.25 m/s when a confirmed hazard, an unresolved candidate or unobserved corridor ground lies within d_stop(1.0) + 0.5 m of the front. d_stop(v) = 0.3 v + v^2 / (2 x 0.5) + 0.10 m is drawn as a line ahead of the front. This gate is a display aid (Assumed numbers); `speed_gate.py` holds the evaluated policy.
- **RViz quirks** worked around: rviz2 (Humble) ignores the window size in the config, so `run_live.sh` resizes the window through Xlib. It also draws spaces in text markers very wide, so the labels use underscores.

## Approach runs on Isaac Sim hits

`./run_isaac_approach.sh [--seed N] [--noise K] [--headless]` exports the arena to USD if needed, captures every pose of `approach.trial(N)` in Isaac Sim, with the window open unless `--headless`, then runs `isaac_approach.py`. That script runs `approach.trial` unchanged twice, once on sandbox scans and once on Isaac hits passed through the BP-1 layer, and scores both with `speed_gate.py`'s rules and the thresholds in params.py. With `--noise K` both runs are repeated over K independent sensor-noise streams, each giving both sources the same draws per frame. `./run_live.sh --source isaac` shows the Isaac-driven run in RViz.

Results (2026-10-12, local ground fit, thresholds 5.1 / 0.85; full tables in `results_isaac/`):

- **Seed 1, 28 frames:** over 10 noise streams, neither source missed a hazard at 1.0 m/s, for the LiDAR or the D455, in clear air or dust. Confirmation frames match in clear air and differ by at most one frame (0.15 m) in dust.
  - LiDAR in dust: phantom stops averaged 0.25 per 10 m on the sandbox and 0 on Isaac, and confirmed false-alarm frames averaged 35.7 vs 31.4.
  - A single noise draw had shown the opposite (Isaac 2.47 per 10 m, 50 vs 25 frames), so that difference was noise, not geometry.
- **Seed 3000, 29 frames, one noise draw:** same confirmation frames except one hazard one frame later with the D455 in dust. No misses or phantom stops, and gate decisions agree in 28-29 of 29 frames.
- **Display:** the GUI changes nothing. A windowed capture of seed 1 matches the headless one exactly: identical ranges and depth, and 0.03 % of rays differ in hit/no-hit through Isaac's random dropout.

## Approach runs and speed

```
python3 approach.py 48 results_approach 3000      # 48 arenas, seeds 3000-3047 (about 35 min on 4 cores)
python3 approach.py 48 results_approach_b 3048    # 48 more for a held-out test
python3 speed_gate.py results_approach results_approach_b   # tune on the first folder, test on the rest
```

`approach.py` drives the robot in a straight line from a random start (x 0.3-0.8 m, y 0.8-4.2 m, heading N(0, 10 deg) clipped to +-25 deg) until its centre reaches x = 4.9 m or comes within 0.5 m of a side wall. The log-odds detector updates every 0.15 m, for the 16-beam LiDAR and the D455, in clear air and with a dust plume 1.2 m ahead. Each frame records every candidate (kind, position, pooled evidence, truth class) and how far ahead the corridor has been observed, so `speed_gate.py` can score any threshold or speed policy without new scans.

`speed_gate.py` starts the robot from rest (0.5 m/s^2 up and down) and counts a hazard as missed when it is first confirmed closer than d_stop(v) = 0.3 v + v^2 / (2 x 0.5) + 0.10 m at the robot's speed in that frame. A phantom stop is a frame where a candidate that is no real obstacle, wall, or view of one (within 0.30 m of an obstacle's edge) reaches the threshold inside that distance. The speed gate drives fast unless unobserved corridor ground or an unconfirmed candidate lies within the fast stopping distance; confirmed hazards go to the planner. All of these numbers are Assumed. It writes `speed_gate.md`, `speed_gate.json`, `sweep_tradeoff.png` and `speed_misses.png`.

Results over seeds 3000-3095 (96 arenas, 68 craters and 111 boulders or column crossings in the path; thresholds 5.1 for LiDAR and 0.85 for D455, set earlier on seeds 1000-1015 and kept by the sweep on 3000-3047):

| Constant speed | Stopping distance | Craters missed, LiDAR | Craters missed, D455 | Boulders missed |
| --- | --- | --- | --- | --- |
| 0.75 m/s | 0.89 m | 0 of 68 | 0 of 68 | 0 of 111 |
| 1.0 m/s | 1.40 m | 1 of 68 | 4 of 68 (dust), 3 (clear) | 0 of 111 |
| 1.25 m/s | 2.04 m | 5 of 68 (dust), 6 (clear) | 13 of 68 (dust), 11 (clear) | 0 of 111 |

LiDAR or D455 together gives the LiDAR's numbers. Phantom stops: 0.17 per 10 m for the LiDAR in dust, 0 otherwise. The speed gate (1.0 / 0.25 m/s, both sensors, dust) averaged 0.50 m/s against 0.66 m/s for a steady 0.75 m/s, spent 26% of the distance slow (20% for unobserved ground) and missed 1 crater, so it does not pay off in this arena (Inferred). Over 0 misses in 68 craters the 95% Wilson interval still reaches 5%.

## Floor and tilt cases

```
python3 approach.py 48 results_rob/base 3000 --clear-only --ground plane            # flat floor, single ground plane
python3 approach.py 48 results_rob/und4 3000 --clear-only --undulation 0.04 --ground plane   # floor waves, 4 cm RMS
python3 approach.py 48 results_rob/und4_local 3000 --clear-only --undulation 0.04   # same, default local ground fit
python3 approach.py 48 results_rob/rms3 3000 --clear-only --rms 0.03 --ground plane # rougher floor (3 cm RMS)
python3 approach.py 48 results_rob/pitch2 3000 --clear-only --pitch-err 2 --ground plane    # attitude estimate off by 2 deg
python3 robustness.py results_rob/summary results_rob/base results_rob/und4 results_rob/und4_local
```

Every case uses the same arenas, starts and sensor noise; the extra draws come from their own random stream. The dust runs draw from the sensor stream too, so the D455 noise is paired only between folders that both run or both skip dust (compare a local-fit folder with dust against `results_approach`, which has dust). `--rms` changes the floor roughness (correlation length 0.25 m, so 3 cm RMS makes bumps about 25 cm wide and up to about 9 cm high). `--undulation` adds 12 long floor waves (wavelengths 1.5-3 m, random directions) of the given RMS inside the walls; the robot still rides level at z = 0, so the waves change what the sensors see, not the robot's attitude. `--pitch-err` rotates every scan about the robot's ground point, as if the attitude estimate were off by that many degrees (the sign alternates with the seed). `robustness.py` scores each folder with the thresholds in `params.py` and the rules of `speed_gate.py` (missed hazards from rest, phantom stops ahead) and adds the false alarms left anywhere in the map per frame. A folder with dust runs is scored twice, clear and in dust. It writes `robustness.md`, `robustness.json` and, with matplotlib, `robustness.png`.

The ground fit is now a robust local plane fit by default (`--ground plane` brings back the single robust plane): LOESS of degree 1 with Tukey bisquare robustness weights (Cleveland 1979, JASA 74(368):829-836, https://stat.cmu.edu/~rnugent/PCMI2016/papers/LowessCleveland.pdf). Each cell's ground is the plane that best fits the observed cell heights around it, weighted by a Gaussian of width `DETECT.ground_sigma`; bisquare weights with c = 4.685 x 1.4826 x MAD of the residuals then remove boulders, craters and noise, and the fit is repeated (4 iterations; Cleveland uses 6 x median|r| and recommends 2). Where under a quarter of the window is observed, the fit leans back towards the global plane. `DETECT.ground_spread` = 0.10 m leaves out cells whose returns span more than 10 cm in height: without it, LiDAR echoes from the dust plume raised the cell mean heights, the local fit followed them up by about 4 cm, and the real ground below read as a crater (1.4 phantom stops per 10 m in dust on the flat floor, against 0.24 with the plane). On seed 3020 ground cells spanned under 6 cm at the 99th percentile and plume cells about 90 cm; the 10 cm limit is Assumed from that. The weighted sums are Gaussian-filtered moment maps, so a pass costs a few image filters.

Results over seeds 3000-3047 (48 arenas, 39 craters and 53 boulder or column crossings in the path), LiDAR and D455 together, constant 0.75 m/s from rest, local fit with a 20 cm window and the 10 cm spread limit:

| Case | False stops per 10 m, plane | False stops per 10 m, local fit | Missed (craters + boulders), plane | Missed, local fit |
| --- | --- | --- | --- | --- |
| Flat floor, 1 cm roughness | 0 | 0 | 0 | 0 |
| Flat floor, dust plume | 0.24 | 0.24 | 0 | 0 |
| Floor waves, 2 cm RMS | 12.8 | 0 | 1 + 0 | 0 |
| Floor waves, 4 cm RMS | 30.9 | 0.10 | 3 + 3 | 0 |
| Roughness 3 cm RMS | 9.0 | 2.4 | 0 | 0 |
| Pitch error 2 deg | 17.5 | 0 | 0 | 0 |
| Floor waves, 1 cm RMS | 1.0 | not run | 0 | not run |
| Roughness 2 cm RMS | 0.78 | not run | 0 | not run |
| Pitch error 1 deg | 0 | not run | 0 | not run |

The LiDAR alone, with the local fit, missed 1 crater on the 4 cm floor waves at 0.75 m/s; the D455 alone missed none. At 1.0 m/s the local fit missed at most 2 of 39 craters with both sensors in every case (the plane: 5 on the 4 cm waves and 4 at 3 cm roughness). The false stops left at 3 cm roughness are bumps up to about 9 cm high and 25 cm wide, which pass the 8 cm boulder height, so some of them are arguably real. Every case here is Assumed: wave amplitudes and wavelengths, roughness and pitch errors were chosen as stress cases, not measured on the arena floor. `robustness.md` in each summary folder has the per-sensor tables.

The earlier results in this README (`results_v02/`, the speed test in `results_approach*/`, the Isaac comparison) were made with the single plane; add `--ground plane` to `approach.py`, or set `DETECT.ground_fit` to `"plane"`, to reproduce them.

## Team robot, Unitree L1, self-mask and ruts

```
./run_selfmask.sh                     # every case below, 48 arenas each (seeds 3000-3047), about 2.5 h on 4 cores
python3 robot.py                      # robot geometry and sensor mounts from the team URDF
python3 approach.py 48 out 3000 --robot --lidar l1 --body table --sensors lidar   # one case
```

`--robot` takes the robot from the team's URDF ([educationmoment/EDT-Lunabotics-2026](https://github.com/educationmoment/EDT-Lunabotics-2026), `sim/robot/uic_bot_2027.xacro` adds the 2027 sensors). The 3D LiDAR is the team's Unitree L1, upside down at the rear top as on the comp-day-one branch, 0.986 m above the ground (Verified from the repo). `sensors.l1_dirs` models its scan from the L1 manual: 21,600 points per second over a 360 x 90 deg hemisphere, so 2,160 per 0.1 s frame (the scan geometry is Inferred). The robot body blocks about 20% of that hemisphere, all forward and down; Isaac Sim agrees with `robot.BodyCaster` on 99.7% of blocked beams (`results_isaac/selfmask_l1.*`).

`selfmask.py` keeps the body out of the map. The table mask stores the body range per direction (0.25 deg grid) and widens it to the farthest body range within 1 deg; a return no farther than that plus 6 cm is dropped. The box mask drops every return inside the robot's bounding box plus 5 cm. `selfmask_eval.py` scores every case with fixed thresholds (L1 6.8, tuned on seeds 3000-3023 in dust; D455 0.85) on seeds 3024-3047.

Results (2026-10-12; all models and thresholds as labelled in params.py):

| Case | Body points per scan in the map | Ground points dropped per scan | Frames with a false obstacle on the robot | False obstacles left on the driven path |
| --- | --- | --- | --- | --- |
| No mask | 421 | 0 | 73% (dust 76%) | 3.5 per approach |
| Box mask | 0 | 190 | 0% (dust 18%) | 0 |
| Table mask | 0.1 | 4.4 | 0% (dust 1%) | 0 |
| Table mask, L1 tilted 1 deg off its mount | 0.2 | 4.2 | 0% | 0 |
| Table mask, L1 tilted 2 deg off its mount | 23.5 | 4.1 | 30% | 3.1 |

So the table mask is the one to use, and the L1's mount angle has to be calibrated to within 1 deg (or the mask widened).

Ruts (`ruts.py`, `--ruts`): six earlier trips weave through the obstacle zone and press ruts into the floor. 1.5 and 3.2 cm ruts change nothing. 5 cm ruts give 20 phantom stops per 10 m for the D455 and 10 for the L1 at 1.0 m/s (clear air): the rut trenches read as craters. Real BP-1 rut depth is unknown (3.2 cm comes from lunar soil values), so it should be measured; if ruts reach 5 cm, the robot should discount crater evidence along its own earlier wheel tracks (Inferred).

The L1 at its real mount confirms hazards late: few of its points land on the corridor ahead (about 33 per scan, against about 480 with the L1 at the front top, its axis 45 deg forward of straight down: `--lidar-mount 0.45,0,0.75,180,-45,0`). `--frame-s` gives each frame the points a robot at speed DS / frame_s collects. Hazards missed (too late to stop), L1 alone, 45 hazards per cell:

| L1 mount | 0.25 m/s | 0.5 m/s | 1.0 m/s |
| --- | --- | --- | --- |
| Rear top, upside down (the team's) | 0 clear, 0 dust (1.3 phantom stops per 10 m in dust) | 7 clear, 17 dust | 33 clear, 38 dust |
| Front top, axis 45 deg forward of down | 0 clear, 0 dust | 0 clear, 0 dust | 2 clear, 6 dust |

Each rear cell uses records with that speed's points per frame (`l1_table_f0.6`, `_f0.3`, `_f0.15`) and a threshold tuned at that speed on seeds 3000-3023 (`speed_gate.py --tune-speed`). The front row uses the 1.0 m/s records (`l1_front45_f0.15`) at every speed, so its slower cells are conservative.

The D455 at its front mount misses 0 of 45 up to 0.75 m/s and 1 of 45 at 1.0 m/s (clear air, `both_noruts`). At the real mount the L1 cannot be the forward hazard sensor above walking pace; the D455 has to carry that, or the L1 moves forward and tilts down, at the cost of its rear view for localization (Inferred).

## Parameter status

| Parameter | Value | Status | Source |
| --- | --- | --- | --- |
| ARENA.length_x | 6.8 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.2.1 |
| ARENA.width_y | 5.0 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.2.1 |
| ARENA.grid_res | 0.01 m | Assumed | sandbox resolution choice |
| ARENA.berm_center | 5.38 to 0.9 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.2.3 |
| ARENA.berm_size | 2.2 to 0.9 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.2.2 (orientation Assumed: 2.2 m along x) |
| ARENA.start_zone | (0.0, 2.0, 0.0, 2.0) m (x0,x1,y0,y1) | Assumed | zone sizes are not in the guidebook text |
| ARENA.obstacle_zone_x | 2.0 to 4.4 m | Assumed | Figure 9 labels unreadable; band chosen between start and berm |
| ARENA.column_xy_r | (3.4, 2.5, 0.15) m | Assumed | 'central support column' (11.4.2); size and position not in text |
| ARENA.surface_rms | 0.01 m | Assumed | raked BP-1 roughness not measured |
| ARENA.surface_corr_len | 0.25 m | Assumed | roughness correlation length |
| ARENA.gravel_per_m2 | 4.0 1/m^2 | Assumed | gravel ~2 cm diameter exists (11.1); density not given |
| ARENA.gravel_diam | 0.02 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.1 item 2 |
| OBSTACLES.n_boulders | 3  | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.4.1 (minimum) |
| OBSTACLES.n_craters | 3  | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.4.2 (minimum) |
| OBSTACLES.boulder_diam | 0.3 to 0.4 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.4.1 |
| OBSTACLES.boulder_height | 0.1 to 0.3 m | Assumed | 11.4.1 says only 'varying heights' |
| OBSTACLES.crater_diam | 0.3 to 0.5 m | Inferred | 11.4.2: depth and width not exceeding ~40-50 cm |
| OBSTACLES.crater_depth | 0.1 to 0.4 m | Assumed | upper bound from 11.4.2; lower bound chosen |
| OBSTACLES.crater_rim_frac | 0.1 of depth | Assumed | rim height; replace with a published crater profile |
| OBSTACLES.min_separation | 0.8 m | Assumed | spacing between obstacle centers |
| BP1.depth | 0.45 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.1 |
| BP1.particle_size_most_under | 0.0001 m | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.1 g |
| BP1.density_compacted | 1.5 to 1.8 g/cm^3 | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.1 f |
| BP1.density_top_raked | 0.75 g/cm^3 | Verified | 2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md) 11.1 h |
| BP1.lidar_reflectivity | 0.1  | Assumed | NOT KNOWN. Guidebook Figure 7 lets teams measure it; needs the PDF figure |
| MOUNTS.d455_xyz | (0.35, 0.0, 0.6) m | Assumed | Angelo: front, 'somewhat high' |
| MOUNTS.d455_pitch_down | 40.0 deg | Verified | Angelo's sensor layout |
| MOUNTS.lidar_xyz | (0.0, 0.0, 0.8) m | Assumed | Angelo: on top of the robot |
| MOUNTS.lidar_pitch_down | 30.0 deg | Assumed | Angelo: 'angled downwards'; angle not given |
| D455.hfov | 87.0 deg | Verified | https://www.intelrealsense.com/depth-camera-d455/ |
| D455.vfov | 58.0 deg | Verified | https://www.intelrealsense.com/depth-camera-d455/ |
| D455.baseline | 0.095 m | Verified | https://www.intelrealsense.com/depth-camera-d455/ |
| D455.native_width | 1280 px | Assumed | focal length is computed at this width |
| D455.sim_size | 212 to 120 px | Assumed | downsampled for speed |
| D455.subpixel_best | 0.08 px | Verified | https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance (curves use subpixel=0.08) |
| D455.subpixel_field | 0.32 px | Inferred | from '<2% at 4 m' with f=674 px, B=0.095 m (step 1 check) |
| D455.min_depth | 0.4 m | Assumed | minimum depth not checked yet |
| D455.max_depth | 6.0 m | Assumed | useful range cap for the sandbox |
| D455.invalid_frac_clear | 0.02  | Assumed | baseline invalid-pixel fraction on low-texture regolith |
| LIDAR_GENERIC.range_sigma | 0.02 m | Assumed | replace with the chosen LiDAR's datasheet |
| LIDAR_GENERIC.max_range_ref | 20.0 m | Assumed | range at which a 10% target at normal incidence is just detected |
| LIDAR_GENERIC.rho_ref | 0.1  | Assumed | reference reflectivity for max_range_ref |
| DUST.beta_peak | 1.5 1/m | Assumed | peak extinction in the dig plume; no BP-1 measurement found yet |
| DUST.beta_background | 0.0 1/m | Assumed | clear air away from digging |
| DUST.plume_sigma_h | 0.5 m | Assumed | horizontal spread |
| DUST.plume_sigma_v | 0.3 m | Assumed | vertical spread |
| DUST.plume_center_z | 0.3 m | Assumed | height of densest dust |
| DUST.echo_kappa | 0.3  | Assumed | fraction of extinction events strong enough to trigger a LiDAR echo |
| DUST.camera_invalid_gain | 1.0  | Assumed | how fast stereo invalid pixels grow as transmittance drops |
| DETECT.cell | 0.05 m | Assumed | elevation grid |
| DETECT.h_pos | 0.08 m | Assumed | positive obstacle threshold above fitted ground |
| DETECT.h_neg | 0.08 m | Assumed | negative obstacle threshold below fitted ground |
| DETECT.min_cells_pos | 3 cells | Assumed |  |
| DETECT.min_cells_neg | 4 cells | Assumed |  |
| DETECT.carve_margin | 0.08 m | Assumed | a ray sample counts as free space only if it is this far (plus 3 sigma) short of the hit |
| DETECT.wall_band | 0.15 m | Assumed | detections this close to the arena edge are walls, scored separately from false alarms |
| DETECT.match_margin | 0.15 m | Assumed | detection counts if its center is within radius + margin |
| DETECT.l_occ | 0.85 | Verified | value from Pairet et al. Sec. 7.1, https://arxiv.org/abs/2004.12317 (our per-ray use on a height grid is Inferred) |
| DETECT.l_free | -0.4 | Verified | Pairet et al. Sec. 7.1 |
| DETECT.l_min | -2.0 | Verified | Pairet et al. Sec. 7.1 |
| DETECT.l_max | 3.5 | Verified | Pairet et al. Sec. 7.1 |
| DETECT.l_decide_lidar | 5.1 | Assumed | LiDAR: accept an object when log-odds summed over its cells reach 6 net observations; tuned on seeds 1000-1015 |
| DETECT.l_decide_d455 | 0.85 | Assumed | D455: accept an object at 1 net observation (it made no dust false alarms); tuned on seeds 1000-1015 |
| DETECT.d_neg | 0.04 m | Assumed | a ray passing this far below the fitted ground (or 3 vertical sigma) is evidence of a depression |
| DETECT.ground_fit | local | Assumed | 'local': robust local planes (Cleveland 1979 LOESS); 'plane': one robust plane for the arena. Local chosen on seeds 3000-3047: same misses on a flat floor, far fewer false stops on floor waves, roughness and pitch error |
| DETECT.ground_sigma | 0.2 m | Assumed | width of the Gaussian window for the local ground fit; 0.2 m beat 0.3 m on 4 cm floor waves (seeds 3000-3003) |
| DETECT.ground_spread | 0.1 m | Assumed | local ground fit uses only cells whose returns span less than this in height (0 = all cells); seed 3020: ground cells under 6 cm at p99, dust-plume cells about 90 cm |
| LIDAR_PROFILES.spin16 | kind=spinning, channels=16, vmin=-15.0, vmax=15.0, hres=0.4, min_range=0.3 | Assumed | Assumed generic 16-beam spinning pattern |
| LIDAR_PROFILES.rosette70 | kind=rosette, fov=70.4, points=10000, petals=7.0, min_range=0.21 | Assumed | Approximate non-repetitive pattern; 70.4 deg FOV and 0.21 m blind zone are the Livox MID-70 values reported by Xie et al. (https://pmc.ncbi.nlm.nih.gov/articles/PMC11679008); the curve itself is Assumed |
