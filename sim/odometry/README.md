# Odometry drift over one dig -> dump -> dig cycle

Step 2 asked how far each odometry source drifts between start-zone tag fixes, and what each part of the fusion adds.
Everything here is simulation (Inferred); numbers marked Assumed in the parameter tables are ours to measure.

## Python test (drift.py)

The robot drives the custom planner's first mission cycle (start -> dig 1 -> dump 1 at the berm -> dig 2, about
7.6 m to the berm and 13 m in all) in the sandbox arenas 3000-3029, at nominal and heavy slip
(sim/planner SLIP_LEVELS). The start pose carries the Isaac two-tag fix error (1.3 cm, 1.2 deg per axis). Sensors:
wheel encoders, a gyro (0.014 deg/s/sqrt(Hz), Inferred from the BMI055; 0.05 deg/s residual bias, Assumed), and the
Unitree L1 at the 2027 front mount with body occlusion, self-mask and motion skew (5 time slices per 0.1 s frame).
The wheel distance scale is calibrated for the mean slip with a 3% per-run calibration error (Inferred). The
controller follows the true pose, so all estimators see the same drive (open loop).

Estimators: `wheel` (encoders only), `wheel_cal` (slip-calibrated), `wheel_gyro`, `lidar` (KISS-ICP 1.3.0 as
shipped), `lidar_gyro` (KISS-ICP with the wheel+gyro motion as its guess and deskew, the IMU's role in FAST-LIO2),
`fused` (wheel_gyro prediction plus lidar_gyro increments fused every 1 s, chi-square gated) and `fused_walls`
(fused plus a four-wall ICP fix every 1 s, localize.icp; three agreeing fits the gate refuses are taken anyway,
which stops the filter from locking out correct fixes after it drifts).

`python3 drift.py results/drift_v2 --seeds 30 --slip nominal,heavy` (about 35 min on 3 cores).
Tuning: sig_win_lidar and the relock rule were set on tuning arenas 1000-1011 (tune.py), not the test arenas.
results/drift_v1 is the same run before the relock rule.

Results, results/drift_v2 (23 and 22 completed round trips; median [95% CI] / 95th percentile):

| Slip | Where | wheel_gyro | lidar_gyro | fused | fused_walls |
| --- | --- | --- | --- | --- | --- |
| nominal | berm, 7.6 m | 14.1 / 35.8 cm | 15.0 / 43.9 cm | 9.1 / 23.1 cm | 0.8 [0.8, 0.9] / 1.2 cm |
| nominal | dig 2, 13.3 m | 13.0 / 37.0 cm | 8.3 / 28.6 cm | 9.3 / 31.1 cm | 0.7 [0.7, 0.8] / 1.6 cm |
| heavy | berm, 7.6 m | 15.5 / 38.1 cm | 12.7 / 35.2 cm | 10.7 / 24.1 cm | 0.8 [0.7, 1.0] / 2.1 cm |
| heavy | dig 2, 13.2 m | 25.7 / 41.2 cm | 8.9 / 19.6 cm | 15.6 / 26.4 cm | 0.7 [0.7, 0.8] / 5.5 cm |

Wheels alone are 50-160 cm off at the berm (skid-steer turns), the gyro fixes heading, and the L1 alone is noisy
frame to frame (about 1 cm and 0.4 deg per frame even standing still) and drifts 2-3 deg in heading during pivots
unless the gyro guides it. Matching the four walls (allowed for localization, guidebook 16.3) is what holds the pose
to about 1 cm; heading stays within 0.2 deg median. Worst case in 90 checks: 12 cm (heavy slip, dig 2).

## Isaac D455 stereo VO (isaac_vo.py, vo_eval.py)

Stereo pairs rendered in Isaac Sim along four of the same drives (results/traj_v1), 848x480, 15 Hz, KLT + stereo
depth + PnP RANSAC, started at the true pose. Diffuse light: 1-5 cm and under 1 deg over the whole cycle. Low sun
(15 deg): 56 cm and 9 deg by dig 2 (278 features median, 23 lost frames); gyro heading cuts that to 26 cm. With the
gyro's 0.05 deg/s bias, gyro heading is worse than the camera's in good light, so the gyro should bridge lost frames
rather than replace VO heading. Ideal camera: no image noise, blur or rectification error. Results in
results/isaac_vo_v1.

## Caveats

Slip, wall height band, wall fix noise and the L1 range model are Assumed; KISS-ICP is a stand-in for FAST-LIO2 /
Point-LIO; VO and LiDAR are not yet fused together; the dust plume and ruts are off in these runs.
