# Approach on Isaac Sim hits vs sandbox (seed 3000)

Start (0.68, 3.65), heading 5.4 deg, 29 frames 0.15 m apart; ground fit local; thresholds LiDAR 5.1, D455 0.85. Rules and Assumed numbers from speed_gate.py.

Hazards in the path, first confirmed at frame (clearance from the robot's front edge, m):

| hazard | LiDAR clear sandbox | LiDAR clear Isaac | LiDAR dust sandbox | LiDAR dust Isaac | D455 clear sandbox | D455 clear Isaac | D455 dust sandbox | D455 dust Isaac |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| boulder 38 cm, 2.79 m ahead | 1 (2.29) | 1 (2.29) | 1 (2.29) | 1 (2.29) | 0 (2.44) | 0 (2.44) | 1 (2.29) | 1 (2.29) |
| crater 42 cm, 1.97 m ahead | 0 (1.62) | 0 (1.62) | 0 (1.62) | 0 (1.62) | 0 (1.62) | 0 (1.62) | 0 (1.62) | 1 (1.47) |

Policies (missed hazards / phantom stops per 10 m):

| case | 0.5 m/s sandbox | 0.5 m/s Isaac | 0.75 m/s sandbox | 0.75 m/s Isaac | 1.0 m/s sandbox | 1.0 m/s Isaac | gate 1.0 sandbox | gate 1.0 Isaac |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| spin16 clear | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| spin16 dust | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| d455 clear | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| d455 dust | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |

Gate decisions per frame (fast/slow at v_fast 1.0, same reasons rule as speed_gate.py):

| case | frames agreeing | sandbox slow frames | Isaac slow frames | confirmed false alarms, sandbox / Isaac (frame-sum) |
| --- | --- | --- | --- | --- |
| spin16 clear | 29/29 | 0 | 0 | 0 / 0 |
| spin16 dust | 28/29 | 2 | 3 | 49 / 55 |
| d455 clear | 29/29 | 1 | 1 | 0 / 0 |
| d455 dust | 29/29 | 1 | 1 | 0 / 0 |
