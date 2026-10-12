# Wheel ruts and false detections (Isaac drive, seed 1, trip seed 2)

40 frames; ruts 5.0 cm vs none; thresholds L1 6.8 (Inferred, cloud sweep), D455 0.85. 'On the ruts': false candidate within 0.15 m of a wheel track. Phantom stop: confirmed false candidate in the corridor within d_stop(1.0) + 0.15 m ahead.

| sensor | case | confirmed false-candidate frames (on ruts) | distinct false candidates (on ruts) | phantom-stop frames (from ruts) | false candidates in the final map |
| --- | --- | --- | --- | --- | --- |
| L1 | with ruts | 153 (138) | 26 (24) | 0 (0) | 9 ['neg', 'neg', 'neg', 'neg', 'neg', 'neg', 'neg', 'neg', 'neg'] |
| L1 | no ruts | 0 (0) | 0 (0) | 0 (0) | 0 [] |
| D455 | with ruts | 0 (0) | 0 (0) | 0 (0) | 0 [] |
| D455 | no ruts | 0 (0) | 0 (0) | 0 (0) | 0 [] |

L1 self-mask (with ruts): 420 body returns per frame before the mask (within 2 cm of robot.BodyCaster's range), 0.1 left after it (max 1).
