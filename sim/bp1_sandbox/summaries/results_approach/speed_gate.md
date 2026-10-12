# Speed gate and threshold sweep (96 approaches)

Thresholds tuned on seeds 3000-3047 (48 arenas): 16-beam LiDAR 5.1, D455 0.85 (the lowest with at most 0.5 phantom stops per 10 m at 1.0 m/s in dust). Everything below is on seeds 3048-3095 (48 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 16-beam LiDAR | clear | craters | 29 | 15 | 14 | 1.87 m | 1.53 m | 0 |
| 16-beam LiDAR | clear | boulders | 58 | 32 | 26 | 2.25 m | 2.18 m | 0 |
| 16-beam LiDAR | dust | craters | 29 | 14 | 15 | 1.87 m | 1.41 m | 0 |
| 16-beam LiDAR | dust | boulders | 58 | 29 | 29 | 2.21 m | 2.14 m | 0 |
| D455 | clear | craters | 29 | 20 | 9 | 1.58 m | 1.37 m | 0 |
| D455 | clear | boulders | 58 | 45 | 13 | 2.51 m | 2.45 m | 0 |
| D455 | dust | craters | 29 | 10 | 19 | 1.52 m | 1.21 m | 0 |
| D455 | dust | boulders | 58 | 33 | 25 | 2.40 m | 2.26 m | 0 |
| LiDAR or D455 | clear | craters | 29 | 20 | 9 | 2.13 m | 1.73 m | 0 |
| LiDAR or D455 | clear | boulders | 58 | 45 | 13 | 2.51 m | 2.45 m | 0 |
| LiDAR or D455 | dust | craters | 29 | 15 | 14 | 1.98 m | 1.45 m | 0 |
| LiDAR or D455 | dust | boulders | 58 | 34 | 24 | 2.40 m | 2.26 m | 0 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 16-beam LiDAR | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| 16-beam LiDAR | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| 16-beam LiDAR | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| 16-beam LiDAR | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| 16-beam LiDAR | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 3/29 (10%) | 0/58 (0%) | 0.00 |
| 16-beam LiDAR | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| 16-beam LiDAR | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| 16-beam LiDAR | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| 16-beam LiDAR | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| 16-beam LiDAR | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 3/29 (10%) | 0/58 (0%) | 0.10 |
| D455 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 1/29 (3%) | 0/58 (0%) | 0.00 |
| D455 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 5/29 (17%) | 0/58 (0%) | 0.00 |
| D455 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| D455 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 1/29 (3%) | 0/58 (0%) | 0.00 |
| D455 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 5/29 (17%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 0/29 (0%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 3/29 (10%) | 0/58 (0%) | 0.00 |
| LiDAR or D455 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| LiDAR or D455 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| LiDAR or D455 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| LiDAR or D455 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 0/29 (0%) | 0/58 (0%) | 0.10 |
| LiDAR or D455 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 3/29 (10%) | 0/58 (0%) | 0.10 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| 16-beam LiDAR | clear | 0.75 m/s | 0.54 m/s | 15% (15% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| 16-beam LiDAR | clear | 1.0 m/s | 0.58 m/s | 21% (21% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| 16-beam LiDAR | clear | 1.25 m/s | 0.47 m/s | 38% (38% / 0%) | 2/87 (2%, 1%-8%) | 0.00 |
| 16-beam LiDAR | dust | 0.75 m/s | 0.47 m/s | 24% (16% / 9%) | 0/87 (0%, 0%-4%) | 0.10 |
| 16-beam LiDAR | dust | 1.0 m/s | 0.46 m/s | 32% (23% / 10%) | 0/87 (0%, 0%-4%) | 0.10 |
| 16-beam LiDAR | dust | 1.25 m/s | 0.41 m/s | 45% (44% / 10%) | 0/87 (0%, 0%-4%) | 0.10 |
| D455 | clear | 0.75 m/s | 0.52 m/s | 17% (17% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| D455 | clear | 1.0 m/s | 0.55 m/s | 23% (23% / 0%) | 1/87 (1%, 0%-6%) | 0.00 |
| D455 | clear | 1.25 m/s | 0.47 m/s | 32% (32% / 0%) | 2/87 (2%, 1%-8%) | 0.00 |
| D455 | dust | 0.75 m/s | 0.52 m/s | 17% (17% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| D455 | dust | 1.0 m/s | 0.56 m/s | 22% (22% / 0%) | 1/87 (1%, 0%-6%) | 0.00 |
| D455 | dust | 1.25 m/s | 0.46 m/s | 33% (33% / 0%) | 2/87 (2%, 1%-8%) | 0.00 |
| LiDAR or D455 | clear | 0.75 m/s | 0.51 m/s | 19% (19% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| LiDAR or D455 | clear | 1.0 m/s | 0.53 m/s | 25% (25% / 0%) | 0/87 (0%, 0%-4%) | 0.00 |
| LiDAR or D455 | clear | 1.25 m/s | 0.44 m/s | 41% (40% / 0%) | 1/87 (1%, 0%-6%) | 0.00 |
| LiDAR or D455 | dust | 0.75 m/s | 0.47 m/s | 25% (19% / 9%) | 0/87 (0%, 0%-4%) | 0.10 |
| LiDAR or D455 | dust | 1.0 m/s | 0.46 m/s | 32% (26% / 10%) | 0/87 (0%, 0%-4%) | 0.10 |
| LiDAR or D455 | dust | 1.25 m/s | 0.41 m/s | 45% (44% / 10%) | 0/87 (0%, 0%-4%) | 0.10 |
