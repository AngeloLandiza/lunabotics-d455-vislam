# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 12.75 (the lowest with at most 0.5 phantom stops per 10 m at 0.5 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 0 | 22 | 0.78 m | 0.35 m | 0 |
| Unitree L1 | clear | boulders | 23 | 4 | 19 | 0.93 m | 0.62 m | 0 |
| Unitree L1 | dust | craters | 22 | 0 | 19 | 0.57 m | 0.23 m | 3 |
| Unitree L1 | dust | boulders | 23 | 1 | 22 | 0.87 m | 0.40 m | 0 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 1/22 (5%) | 1/23 (4%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 5/22 (23%) | 2/23 (9%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 11/22 (50%) | 6/23 (26%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 17/22 (77%) | 9/23 (39%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 17/22 (77%) | 9/23 (39%) | 0.00 |
| Unitree L1 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 5/22 (23%) | 0/23 (0%) | 0.20 |
| Unitree L1 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 11/22 (50%) | 6/23 (26%) | 0.49 |
| Unitree L1 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 20/22 (91%) | 11/23 (48%) | 0.69 |
| Unitree L1 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 21/22 (95%) | 15/23 (65%) | 0.69 |
| Unitree L1 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 21/22 (95%) | 15/23 (65%) | 0.69 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.43 m/s | 31% (27% / 5%) | 10/45 (22%, 13%-36%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.39 m/s | 48% (38% / 14%) | 4/45 (9%, 4%-21%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.35 m/s | 69% (59% / 23%) | 2/45 (4%, 1%-15%) | 0.00 |
| Unitree L1 | dust | 0.75 m/s | 0.36 m/s | 48% (37% / 33%) | 11/45 (24%, 14%-39%) | 0.20 |
| Unitree L1 | dust | 1.0 m/s | 0.35 m/s | 58% (48% / 38%) | 6/45 (13%, 6%-26%) | 0.20 |
| Unitree L1 | dust | 1.25 m/s | 0.33 m/s | 72% (64% / 40%) | 6/45 (13%, 6%-26%) | 0.20 |
