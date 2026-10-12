# RTAB-Map relocalization before and after wheel ruts (team settings, Isaac images)

Map: pass A (no ruts). Revisits start at frames [10, 25] with odometry restarted there (ground-truth odometry, so only place recognition is tested). Team parameters from src/config_pkg/params/rtabmap/rtabmap_params.yaml (comp-day-one); overrides and defaults listed in rtabmap_reloc.json.

Pose error is planar (x, y; the team runs Reg/Force3DoF true). Wrong relocalization: an accepted loop closure whose localization error exceeds 0.5 m.

| light | revisit pass | start | frames relocalized | first relocalization (frames after start) | mean inliers | pose error median / p90 (m) | wrong relocalizations |
| --- | --- | --- | --- | --- | --- | --- | --- |
| diffuse | Arep | 10 | 23/30 | 1 | 687 | 0.003 / 0.003 | 0 |
| diffuse | Arep | 25 | 12/15 | 1 | 696 | 0.003 / 0.003 | 0 |
| diffuse | Acam32 | 10 | 27/30 | 1 | 431 | 0.005 / 0.005 | 0 |
| diffuse | Acam32 | 25 | 14/15 | 1 | 419 | 0.005 / 0.005 | 0 |
| diffuse | B32 | 10 | 27/30 | 1 | 344 | 0.005 / 0.006 | 0 |
| diffuse | B32 | 25 | 14/15 | 1 | 341 | 0.005 / 0.005 | 0 |
| diffuse | B32cam | 10 | 24/30 | 1 | 561 | 0.003 / 0.003 | 0 |
| diffuse | B32cam | 25 | 12/15 | 1 | 571 | 0.002 / 0.003 | 0 |
| diffuse | B50 | 10 | 26/30 | 1 | 211 | 0.004 / 0.006 | 0 |
| diffuse | B50 | 25 | 14/15 | 1 | 190 | 0.004 / 0.006 | 0 |
| low | Arep | 10 | 19/30 | 1 | 420 | 0.008 / 0.019 | 0 |
| low | Arep | 25 | 10/15 | 1 | 452 | 0.017 / 0.024 | 0 |
| low | Acam32 | 10 | 27/30 | 1 | 183 | 0.009 / 0.020 | 0 |
| low | Acam32 | 25 | 14/15 | 1 | 187 | 0.017 / 0.024 | 0 |
| low | B32 | 10 | 26/30 | 2 | 156 | 0.012 / 0.021 | 0 |
| low | B32 | 25 | 14/15 | 1 | 160 | 0.018 / 0.023 | 0 |
| low | B32cam | 10 | 23/30 | 1 | 336 | 0.006 / 0.019 | 0 |
| low | B32cam | 25 | 7/15 | 6 | 390 | 0.019 / 0.781 | 0 |
| low | B50 | 10 | 22/30 | 6 | 111 | 0.018 / 0.024 | 0 |
| low | B50 | 25 | 14/15 | 1 | 99 | 0.021 / 0.024 | 0 |
