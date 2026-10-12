# Speed gate and threshold sweep (48 approaches)

Thresholds tuned on seeds 3000-3023 (24 arenas): Unitree L1 5.1 (the lowest with at most 0.5 phantom stops per 10 m at 0.25 m/s in dust, or in clear air when the records have no dust). Everything below is on seeds 3024-3047 (24 arenas), not used for tuning.
The robot starts from rest and speeds up or brakes at 0.5 / 0.5 m/s^2. Stopping distance d_stop(v) = v x 0.3 s + v^2 / (2 x 0.5 m/s^2) + 0.1 m. Candidates within 0.30 m of a real obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every 0.15 m; corridor half-width 0.4029832661151886 m; dust plume 1.2 m ahead of the start.

## How far ahead of the front each hazard is first confirmed

Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put them, not how far the sensor reaches.

| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | 10th percentile | Never |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | craters | 22 | 1 | 21 | 1.25 m | 0.76 m | 0 |
| Unitree L1 | clear | boulders | 23 | 10 | 13 | 1.94 m | 1.47 m | 0 |
| Unitree L1 | dust | craters | 22 | 1 | 21 | 0.94 m | 0.56 m | 0 |
| Unitree L1 | dust | boulders | 23 | 10 | 13 | 1.52 m | 0.99 m | 0 |

## Constant speed: missed hazards and phantom stops

| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 0.5 m/s | 0.50 m | 0.47 m/s | 0/22 (0%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 0.75 m/s | 0.89 m | 0.66 m/s | 1/22 (5%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 1.40 m | 0.81 m/s | 4/22 (18%) | 0/23 (0%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 2.04 m | 0.91 m/s | 4/22 (18%) | 0/23 (0%) | 0.00 |
| Unitree L1 | dust | 0.25 m/s | 0.24 m | 0.24 m/s | 0/22 (0%) | 0/23 (0%) | 1.27 |
| Unitree L1 | dust | 0.5 m/s | 0.50 m | 0.47 m/s | 1/22 (5%) | 0/23 (0%) | 7.34 |
| Unitree L1 | dust | 0.75 m/s | 0.89 m | 0.66 m/s | 7/22 (32%) | 1/23 (4%) | 7.83 |
| Unitree L1 | dust | 1.0 m/s | 1.40 m | 0.81 m/s | 16/22 (73%) | 3/23 (13%) | 7.83 |
| Unitree L1 | dust | 1.25 m/s | 2.04 m | 0.91 m/s | 17/22 (77%) | 5/23 (22%) | 7.83 |

## Speed gate: fast while nothing is unresolved within the fast stopping distance, else 0.25 m/s

| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | Missed hazards (95% interval) | Phantom stops per 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Unitree L1 | clear | 0.75 m/s | 0.53 m/s | 15% (15% / 0%) | 1/45 (2%, 0%-12%) | 0.00 |
| Unitree L1 | clear | 1.0 m/s | 0.52 m/s | 24% (23% / 1%) | 3/45 (7%, 2%-18%) | 0.00 |
| Unitree L1 | clear | 1.25 m/s | 0.48 m/s | 39% (37% / 3%) | 1/45 (2%, 0%-12%) | 0.00 |
| Unitree L1 | dust | 0.75 m/s | 0.39 m/s | 39% (25% / 28%) | 0/45 (0%, 0%-8%) | 1.27 |
| Unitree L1 | dust | 1.0 m/s | 0.40 m/s | 43% (34% / 29%) | 0/45 (0%, 0%-8%) | 1.27 |
| Unitree L1 | dust | 1.25 m/s | 0.38 m/s | 56% (48% / 31%) | 0/45 (0%, 0%-8%) | 1.27 |
