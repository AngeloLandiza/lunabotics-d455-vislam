# Landmarks before and after wheel ruts (Isaac, seed 1, trip 2, 40 frames, D455-like IR camera 848x480)

ORB (2000 max, FAST threshold 7); a relocalization match is correct when it reprojects within 3.0 px using the known poses. Survival: share of the map's (pass A) visible features still matched correctly in pass B at the same frame. Near rut edge: feature's ground point within 3 cm of a trench edge.

Off-rut survival: the same for map features more than 0.15 m from every wheel track.

| light | pass | features per image | near a rut edge | survival vs A (mean / worst frame) | off-rut survival |
| --- | --- | --- | --- | --- | --- |
| diffuse | A: no ruts (the map) | 1549 | 10.3 % | - | - |
| diffuse | control: A rendered again | 1550 | 10.1 % | 56 % / 35 % | 56 % |
| diffuse | control: no ruts, B32's camera poses | 1555 | 10.6 % | 39 % / 25 % | 40 % |
| diffuse | B: 3.2 cm ruts, robot in them | 1719 | 45.1 % | 19 % / 4 % | 22 % |
| diffuse | B: 3.2 cm ruts, A's camera poses | 1716 | 44.0 % | 26 % / 4 % | 31 % |
| diffuse | B: 5 cm ruts, robot in them | 1761 | 54.0 % | 15 % / 0 % | 17 % |
| low | A: no ruts (the map) | 172 | 4.5 % | - | - |
| low | control: A rendered again | 221 | 8.2 % | 38 % / 14 % | 40 % |
| low | control: no ruts, B32's camera poses | 186 | 7.7 % | 23 % / 8 % | 23 % |
| low | B: 3.2 cm ruts, robot in them | 383 | 57.1 % | 18 % / 2 % | 19 % |
| low | B: 3.2 cm ruts, A's camera poses | 402 | 52.4 % | 28 % / 0 % | 29 % |
| low | B: 5 cm ruts, robot in them | 554 | 67.2 % | 15 % / 0 % | 16 % |
