# Floor roughness, floor waves and attitude error

Thresholds from params.py (LiDAR 5.1, D455 0.85), constant speed from rest; same arenas and starts in every case, and the same sensor noise among folders that all skip or all run dust.

| Case | Sensor | False boulders in map per frame | False craters in map per frame | Missed at 0.75 m/s (craters, boulders) | Phantom stops per 10 m at 0.75 m/s | Missed at 1.0 m/s (craters, boulders) | Phantom stops per 10 m at 1.0 m/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| baseline (1 cm roughness, flat, exact attitude) | 16-beam LiDAR | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude) | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude) | LiDAR or D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), dust plume | 16-beam LiDAR | 1.22 | 0.00 | 0/39, 0/53 | 0.24 | 1/39, 0/53 | 0.24 |
| baseline (1 cm roughness, flat, exact attitude), dust plume | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 3/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), dust plume | LiDAR or D455 | 0.61 | 0.00 | 0/39, 0/53 | 0.24 | 1/39, 0/53 | 0.24 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread) | 16-beam LiDAR | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 0/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread) | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread) | LiDAR or D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 0/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread), dust plume | 16-beam LiDAR | 1.22 | 0.00 | 0/39, 0/53 | 0.24 | 2/39, 0/53 | 0.24 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread), dust plume | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window, cells under 10 cm spread), dust plume | LiDAR or D455 | 0.61 | 0.00 | 0/39, 0/53 | 0.24 | 1/39, 0/53 | 0.24 |
| floor waves 1 cm | 16-beam LiDAR | 0.00 | 0.10 | 0/39, 0/53 | 0.92 | 1/39, 0/53 | 1.07 |
| floor waves 1 cm | D455 | 0.00 | 0.04 | 0/39, 0/53 | 0.10 | 2/39, 0/53 | 0.19 |
| floor waves 1 cm | LiDAR or D455 | 0.00 | 0.07 | 0/39, 0/53 | 1.02 | 1/39, 0/53 | 1.12 |
| floor waves 2 cm | 16-beam LiDAR | 0.19 | 3.14 | 1/39, 2/53 | 12.43 | 1/39, 2/53 | 15.78 |
| floor waves 2 cm | D455 | 0.03 | 1.63 | 2/39, 0/53 | 5.29 | 2/39, 0/53 | 9.03 |
| floor waves 2 cm | LiDAR or D455 | 0.11 | 2.39 | 1/39, 0/53 | 12.82 | 1/39, 0/53 | 16.61 |
| floor waves 2 cm, local ground fit (20 cm window, cells under 10 cm spread) | 16-beam LiDAR | 0.00 | 0.13 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| floor waves 2 cm, local ground fit (20 cm window, cells under 10 cm spread) | D455 | 0.00 | 0.02 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| floor waves 2 cm, local ground fit (20 cm window, cells under 10 cm spread) | LiDAR or D455 | 0.00 | 0.07 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| floor waves 4 cm | 16-beam LiDAR | 2.80 | 6.26 | 4/39, 5/53 | 25.78 | 8/39, 5/53 | 35.06 |
| floor waves 4 cm | D455 | 1.78 | 3.90 | 5/39, 4/53 | 25.88 | 10/39, 4/53 | 35.83 |
| floor waves 4 cm | LiDAR or D455 | 2.29 | 5.08 | 3/39, 3/53 | 30.88 | 5/39, 3/53 | 40.64 |
| floor waves 4 cm, local ground fit (20 cm window, cells under 10 cm spread) | 16-beam LiDAR | 0.05 | 1.99 | 1/39, 0/53 | 0.10 | 2/39, 0/53 | 1.65 |
| floor waves 4 cm, local ground fit (20 cm window, cells under 10 cm spread) | D455 | 0.04 | 0.59 | 0/39, 0/53 | 0.00 | 3/39, 0/53 | 0.05 |
| floor waves 4 cm, local ground fit (20 cm window, cells under 10 cm spread) | LiDAR or D455 | 0.04 | 1.29 | 0/39, 0/53 | 0.10 | 2/39, 0/53 | 1.65 |
| roughness 2 cm | 16-beam LiDAR | 0.02 | 0.00 | 0/39, 0/53 | 0.78 | 2/39, 0/53 | 0.78 |
| roughness 2 cm | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 4/39, 0/53 | 0.00 |
| roughness 2 cm | LiDAR or D455 | 0.01 | 0.00 | 0/39, 0/53 | 0.78 | 2/39, 0/53 | 0.78 |
| roughness 3 cm | 16-beam LiDAR | 1.31 | 0.00 | 0/39, 0/53 | 8.50 | 4/39, 0/53 | 9.42 |
| roughness 3 cm | D455 | 0.89 | 0.00 | 0/39, 0/53 | 4.47 | 5/39, 0/53 | 6.36 |
| roughness 3 cm | LiDAR or D455 | 1.10 | 0.00 | 0/39, 0/53 | 9.03 | 4/39, 0/53 | 10.49 |
| roughness 3 cm, local ground fit (20 cm window, cells under 10 cm spread) | 16-beam LiDAR | 0.30 | 0.00 | 0/39, 0/53 | 2.14 | 2/39, 0/53 | 2.33 |
| roughness 3 cm, local ground fit (20 cm window, cells under 10 cm spread) | D455 | 0.21 | 0.00 | 0/39, 0/53 | 1.41 | 4/39, 0/53 | 1.89 |
| roughness 3 cm, local ground fit (20 cm window, cells under 10 cm spread) | LiDAR or D455 | 0.25 | 0.00 | 0/39, 0/53 | 2.38 | 2/39, 0/53 | 2.72 |
| pitch error 1 deg | 16-beam LiDAR | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| pitch error 1 deg | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| pitch error 1 deg | LiDAR or D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 1/39, 0/53 | 0.00 |
| pitch error 2 deg | 16-beam LiDAR | 0.02 | 2.96 | 0/39, 0/53 | 14.76 | 2/39, 0/53 | 14.76 |
| pitch error 2 deg | D455 | 0.00 | 0.48 | 0/39, 0/53 | 10.20 | 3/39, 0/53 | 10.25 |
| pitch error 2 deg | LiDAR or D455 | 0.01 | 1.72 | 0/39, 0/53 | 17.48 | 1/39, 0/53 | 17.53 |
| pitch error 2 deg, local ground fit (20 cm window, cells under 10 cm spread) | 16-beam LiDAR | 0.00 | 0.75 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| pitch error 2 deg, local ground fit (20 cm window, cells under 10 cm spread) | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 3/39, 0/53 | 0.00 |
| pitch error 2 deg, local ground fit (20 cm window, cells under 10 cm spread) | LiDAR or D455 | 0.00 | 0.38 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window) | 16-beam LiDAR | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 0/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window) | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 2/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window) | LiDAR or D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 0/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window), dust plume | 16-beam LiDAR | 1.22 | 0.15 | 0/39, 0/53 | 1.36 | 2/39, 0/53 | 1.36 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window), dust plume | D455 | 0.00 | 0.00 | 0/39, 0/53 | 0.00 | 3/39, 0/53 | 0.00 |
| baseline (1 cm roughness, flat, exact attitude), local ground fit (20 cm window), dust plume | LiDAR or D455 | 0.61 | 0.07 | 0/39, 0/53 | 1.36 | 2/39, 0/53 | 1.36 |
| floor waves 4 cm, local ground fit (15 cm window) | 16-beam LiDAR | 0.06 | 2.09 | 1/39, 0/53 | 0.15 | 2/39, 0/53 | 1.55 |
| floor waves 4 cm, local ground fit (15 cm window) | D455 | 0.04 | 0.65 | 0/39, 0/53 | 0.00 | 4/39, 0/53 | 0.00 |
| floor waves 4 cm, local ground fit (15 cm window) | LiDAR or D455 | 0.05 | 1.37 | 0/39, 0/53 | 0.15 | 2/39, 0/53 | 1.55 |
