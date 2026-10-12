# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 6.8 (the lowest with at most 0.5 phantom stops per 10 m at 1.0 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 0 | 15 | 0.53 m | 0.28 m | 7 |
| Unitree L1 | clear | boulders | 23 | 1 | 19 | 0.55 m | 0.26 m | 3 |
| Unitree L1 | dust | craters | 22 | 0 | 15 | 0.35 m | 0.23 m | 7 |
| Unitree L1 | dust | boulders | 23 | 2 | 18 | 0.51 m | 0.24 m | 3 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 8/22 (36%) | 4/23 (17%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 13/22 (59%) | 10/23 (43%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 22/22 (100%) | 15/23 (65%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 22/22 (100%) | 16/23 (70%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 22/22 (100%) | 17/23 (74%) | 0.00 |
| Unitree L1 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 9/22 (41%) | 5/23 (22%) | 0.39 |
| Unitree L1 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 20/22 (91%) | 12/23 (52%) | 0.39 |
| Unitree L1 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 22/22 (100%) | 16/23 (70%) | 0.39 |
| Unitree L1 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 22/22 (100%) | 17/23 (74%) | 0.39 |
| Unitree L1 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 22/22 (100%) | 17/23 (74%) | 0.39 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.28 m/s | 78% (75% / 12%) | 12/45 (27%, 16%-41%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.27 m/s | 86% (85% / 19%) | 12/45 (27%, 16%-41%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.25 m/s | 97% (97% / 22%) | 12/45 (27%, 16%-41%) | 0.00 |
| Unitree L1 | dust | 0.75 m/s | 0.28 m/s | 75% (73% / 31%) | 15/45 (33%, 21%-48%) | 0.39 |
| Unitree L1 | dust | 1.0 m/s | 0.27 m/s | 86% (84% / 33%) | 14/45 (31%, 20%-46%) | 0.39 |
| Unitree L1 | dust | 1.25 m/s | 0.25 m/s | 97% (96% / 34%) | 14/45 (31%, 20%-46%) | 0.39 |
