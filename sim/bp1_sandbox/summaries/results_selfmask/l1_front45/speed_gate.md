# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 6.8 (the lowest with at most 0.5 phantom stops per 10 m at 1.0 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 5 | 17 | 1.39 m | 0.93 m | 0 |
| Unitree L1 | clear | boulders | 23 | 10 | 13 | 1.67 m | 1.30 m | 0 |
| Unitree L1 | dust | craters | 22 | 2 | 20 | 1.15 m | 0.90 m | 0 |
| Unitree L1 | dust | boulders | 23 | 8 | 15 | 1.25 m | 0.91 m | 0 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 6/22 (27%) | 1/23 (4%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 7/22 (32%) | 5/23 (22%) | 0.00 |
| Unitree L1 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 0.29 |
| Unitree L1 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 1/22 (5%) | 0/23 (0%) | 0.39 |
| Unitree L1 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 1/22 (5%) | 0/23 (0%) | 0.39 |
| Unitree L1 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 5/22 (23%) | 5/23 (22%) | 0.39 |
| Unitree L1 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 9/22 (41%) | 7/23 (30%) | 0.39 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.55 m/s | 12% (12% / 0%) | 0/45 (0%, 0%-8%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.45 m/s | 35% (34% / 2%) | 4/45 (9%, 4%-21%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.37 m/s | 63% (61% / 9%) | 0/45 (0%, 0%-8%) | 0.00 |
| Unitree L1 | dust | 0.75 m/s | 0.40 m/s | 35% (16% / 28%) | 0/45 (0%, 0%-8%) | 0.29 |
| Unitree L1 | dust | 1.0 m/s | 0.39 m/s | 47% (43% / 30%) | 1/45 (2%, 0%-12%) | 0.29 |
| Unitree L1 | dust | 1.25 m/s | 0.33 m/s | 73% (72% / 32%) | 0/45 (0%, 0%-8%) | 0.29 |
