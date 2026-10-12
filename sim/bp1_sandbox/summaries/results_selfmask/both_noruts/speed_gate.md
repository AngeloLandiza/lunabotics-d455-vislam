# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 0.85, D455 0.85 (the lowest with at most 0.5 phantom stops per 10 m at 1.0 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 0 | 18 | 0.77 m | 0.37 m | 4 |
| Unitree L1 | clear | boulders | 23 | 5 | 18 | 1.08 m | 0.54 m | 0 |
| D455 | clear | craters | 22 | 15 | 7 | 1.99 m | 1.50 m | 0 |
| D455 | clear | boulders | 23 | 17 | 6 | 2.65 m | 2.56 m | 0 |
| LiDAR or D455 | clear | craters | 22 | 15 | 7 | 1.99 m | 1.50 m | 0 |
| LiDAR or D455 | clear | boulders | 23 | 17 | 6 | 2.65 m | 2.56 m | 0 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 5/22 (23%) | 1/23 (4%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 8/22 (36%) | 2/23 (9%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 13/22 (59%) | 5/23 (22%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 18/22 (82%) | 8/23 (35%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 19/22 (86%) | 9/23 (39%) | 0.00 |
| D455 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| D455 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| D455 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| D455 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 1/22 (5%) | 0/23 (0%) | 0.20 |
| D455 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 2/22 (9%) | 0/23 (0%) | 0.20 |
| LiDAR or D455 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| LiDAR or D455 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| LiDAR or D455 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| LiDAR or D455 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 1/22 (5%) | 0/23 (0%) | 0.20 |
| LiDAR or D455 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 2/22 (9%) | 0/23 (0%) | 0.20 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.29 m/s | 75% (75% / 0%) | 7/45 (16%, 8%-29%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.27 m/s | 85% (85% / 0%) | 7/45 (16%, 8%-29%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.25 m/s | 97% (97% / 0%) | 7/45 (16%, 8%-29%) | 0.00 |
| D455 | clear | 0.75 m/s | 0.55 m/s | 11% (11% / 0%) | 0/45 (0%, 0%-8%) | 0.00 |
| D455 | clear | 1.0 m/s | 0.60 m/s | 15% (15% / 0%) | 1/45 (2%, 0%-12%) | 0.20 |
| D455 | clear | 1.25 m/s | 0.59 m/s | 18% (18% / 0%) | 2/45 (4%, 1%-15%) | 0.20 |
| LiDAR or D455 | clear | 0.75 m/s | 0.29 m/s | 75% (75% / 0%) | 0/45 (0%, 0%-8%) | 0.00 |
| LiDAR or D455 | clear | 1.0 m/s | 0.27 m/s | 85% (85% / 0%) | 0/45 (0%, 0%-8%) | 0.00 |
| LiDAR or D455 | clear | 1.25 m/s | 0.25 m/s | 97% (97% / 0%) | 0/45 (0%, 0%-8%) | 0.00 |
