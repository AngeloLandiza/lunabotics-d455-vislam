# Self-mask, calibration error and ruts

Thresholds from results_selfmask/l1_table/speed_gate.json: Unitree L1 6.8, D455 0.85. Scored on seeds from 3024 up, which the thresholds were not tuned on. Footprint phantoms count false candidates at or above the threshold within 0.1 m of the robot's footprint.

| Case | Sensor | Air | Approaches | Body points per frame | Ground removed per frame | Blind ahead (median / 90th pct) | Frames with a phantom on the robot | Phantoms left on the path | Missed at 1.0 m/s | Phantom stops per 10 m at 1.0 m/s | Gate mean speed (missed) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| l1_none | Unitree L1 | clear | 24 | 0.0 | 0.0 | 0.01 m / 0.02 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| l1_none | Unitree L1 | dust | 24 | 0.0 | 0.0 | 0.01 m / 0.02 m | 0% | 0.00 | 39/45 (87%) | 0.20 | 0.28 m/s (15/45) |
| l1_raw | Unitree L1 | clear | 24 | 421.1 | 0.0 | 0.19 m / 0.34 m | 73% | 3.54 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| l1_raw | Unitree L1 | dust | 24 | 400.3 | 0.0 | 0.19 m / 0.34 m | 76% | 3.75 | 39/45 (87%) | 0.49 | 0.27 m/s (14/45) |
| l1_box | Unitree L1 | clear | 24 | 0.0 | 192.5 | 0.19 m / 0.34 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| l1_box | Unitree L1 | dust | 24 | 0.0 | 179.2 | 0.19 m / 0.35 m | 18% | 0.00 | 39/45 (87%) | 0.39 | 0.27 m/s (14/45) |
| l1_table | Unitree L1 | clear | 24 | 0.1 | 4.4 | 0.19 m / 0.34 m | 0% | 0.00 | 38/45 (84%) | 0.00 | 0.27 m/s (12/45) |
| l1_table | Unitree L1 | dust | 24 | 0.1 | 4.2 | 0.19 m / 0.34 m | 1% | 0.00 | 39/45 (87%) | 0.39 | 0.27 m/s (14/45) |
| l1_table_tilt1 | Unitree L1 | clear | 24 | 0.2 | 4.2 | 0.17 m / 0.33 m | 0% | 0.00 | 37/45 (82%) | 0.00 | 0.27 m/s (14/45) |
| l1_table_tilt2 | Unitree L1 | clear | 24 | 23.5 | 4.1 | 0.15 m / 0.30 m | 30% | 3.12 | 38/45 (84%) | 0.00 | 0.27 m/s (16/45) |
| l1_box_tilt1 | Unitree L1 | clear | 24 | 0.0 | 189.2 | 0.17 m / 0.33 m | 0% | 0.00 | 37/45 (82%) | 0.00 | 0.27 m/s (14/45) |
