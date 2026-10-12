# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 8.5 (the lowest with at most 0.5 phantom stops per 10 m at 1.0 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 0 | 21 | 0.57 m | 0.17 m | 1 |
| Unitree L1 | clear | boulders | 23 | 2 | 19 | 0.86 m | 0.52 m | 2 |
| Unitree L1 | dust | craters | 22 | 0 | 19 | 0.35 m | 0.08 m | 3 |
| Unitree L1 | dust | boulders | 23 | 2 | 19 | 0.78 m | 0.40 m | 2 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 5/22 (23%) | 3/23 (13%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 8/22 (36%) | 4/23 (17%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 18/22 (82%) | 9/23 (39%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 20/22 (91%) | 13/23 (57%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 20/22 (91%) | 14/23 (61%) | 0.00 |
| Unitree L1 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 8/22 (36%) | 2/23 (9%) | 0.29 |
| Unitree L1 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 17/22 (77%) | 8/23 (35%) | 0.29 |
| Unitree L1 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 22/22 (100%) | 13/23 (57%) | 0.29 |
| Unitree L1 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 22/22 (100%) | 16/23 (70%) | 0.29 |
| Unitree L1 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 22/22 (100%) | 16/23 (70%) | 0.29 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.36 m/s | 50% (43% / 12%) | 14/45 (31%, 20%-46%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.33 m/s | 64% (57% / 19%) | 9/45 (20%, 11%-34%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.29 m/s | 83% (80% / 24%) | 8/45 (18%, 9%-31%) | 0.00 |
| Unitree L1 | dust | 0.75 m/s | 0.32 m/s | 59% (55% / 31%) | 13/45 (29%, 18%-43%) | 0.29 |
| Unitree L1 | dust | 1.0 m/s | 0.32 m/s | 67% (63% / 35%) | 11/45 (24%, 14%-39%) | 0.29 |
| Unitree L1 | dust | 1.25 m/s | 0.29 m/s | 84% (82% / 36%) | 11/45 (24%, 14%-39%) | 0.29 |
