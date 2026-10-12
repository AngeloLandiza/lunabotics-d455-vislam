# Approach on Isaac Sim hits vs sandbox (seed 1)

Start (0.76, 1.33), heading 0.5 deg, 28 frames 0.15 m apart; ground fit local; thresholds LiDAR 5.1, D455 0.85. Rules and Assumed numbers from speed_gate.py.

Hazards in the path, first confirmed at frame (clearance from the robot's front edge, m):

| hazard | LiDAR clear sandbox | LiDAR clear Isaac | LiDAR dust sandbox | LiDAR dust Isaac | D455 clear sandbox | D455 clear Isaac | D455 dust sandbox | D455 dust Isaac |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| boulder 35 cm, 2.77 m ahead | 1 (2.27) | 1 (2.27) | 2 (2.12) | 1 (2.27) | 0 (2.42) | 0 (2.42) | 1 (2.27) | 1 (2.27) |
| boulder 34 cm, 1.45 m ahead | 0 (1.10) | 0 (1.10) | 0 (1.10) | 0 (1.10) | 0 (1.10) | 0 (1.10) | 0 (1.10) | 0 (1.10) |
| crater 41 cm, 1.98 m ahead | 0 (1.63) | 0 (1.63) | 0 (1.63) | 0 (1.63) | 0 (1.63) | 0 (1.63) | 3 (1.18) | 1 (1.48) |

Policies (missed hazards / phantom stops per 10 m):

| case | 0.5 m/s sandbox | 0.5 m/s Isaac | 0.75 m/s sandbox | 0.75 m/s Isaac | 1.0 m/s sandbox | 1.0 m/s Isaac | gate 1.0 sandbox | gate 1.0 Isaac |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| spin16 clear | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| spin16 dust | 0 / 0.00 | 0 / 2.47 | 0 / 0.00 | 0 / 2.47 | 0 / 0.00 | 0 / 2.47 | 0 / 0.00 | 0 / 2.47 |
| d455 clear | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| d455 dust | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |

Gate decisions per frame (fast/slow at v_fast 1.0, same reasons rule as speed_gate.py):

| case | frames agreeing | sandbox slow frames | Isaac slow frames | confirmed false alarms, sandbox / Isaac (frame-sum) |
| --- | --- | --- | --- | --- |
| spin16 clear | 28/28 | 0 | 0 | 0 / 0 |
| spin16 dust | 25/28 | 2 | 5 | 25 / 50 |
| d455 clear | 28/28 | 1 | 1 | 0 / 0 |
| d455 dust | 28/28 | 1 | 1 | 0 / 0 |
