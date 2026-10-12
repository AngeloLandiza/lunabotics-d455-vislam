# Self-mask, calibration error and ruts

Thresholds from results_selfmask/l1_table/speed_gate.json: Unitree L1 6.8, D455 0.85. Scored on seeds from 3024 up, which the thresholds were not tuned on. Footprint phantoms count false candidates at or above the threshold within 0.1 m of the robot's footprint.

| Case | Sensor | Air | Approaches | Body points per frame | Ground removed per frame | Blind ahead (median / 90th pct) | Frames with a phantom on the robot | Phantoms left on the path | Missed at 1.0 m/s | Phantom stops per 10 m at 1.0 m/s | Gate mean speed (missed) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| both_noruts | Unitree L1 | clear | 24 | 0.1 | 4.4 | 0.19 m / 0.34 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| both_noruts | D455 | clear | 24 | 0.0 | 0.0 | 0.22 m / 0.22 m | 0% | 0.00 | 1/45 (2%) | 0.20 | 0.60 m/s (1/45) |
| both_ruts1.5 | Unitree L1 | clear | 24 | 0.1 | 4.4 | 0.19 m / 0.35 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| both_ruts1.5 | D455 | clear | 24 | 0.0 | 0.0 | 0.22 m / 0.22 m | 0% | 0.00 | 1/45 (2%) | 0.20 | 0.60 m/s (1/45) |
| both_ruts3.2 | Unitree L1 | clear | 24 | 0.1 | 4.4 | 0.19 m / 0.35 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| both_ruts3.2 | D455 | clear | 24 | 0.0 | 0.0 | 0.22 m / 0.22 m | 0% | 0.00 | 1/45 (2%) | 0.59 | 0.60 m/s (1/45) |
| both_ruts5 | Unitree L1 | clear | 24 | 0.1 | 4.3 | 0.19 m / 0.36 m | 15% | 1.79 | 38/45 (84%) | 10.08 | 0.25 m/s (12/45) |
| both_ruts5 | D455 | clear | 24 | 0.0 | 0.0 | 0.21 m / 0.22 m | 20% | 0.79 | 1/45 (2%) | 19.97 | 0.60 m/s (1/45) |
