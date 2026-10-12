# Landmarks before and after wheel ruts (Isaac, seed 1, trip 2, 40 frames, D455-like IR camera 848x480)

ORB (2000 max, FAST threshold 7); a relocalization match is correct when it reprojects within 3.0 px using the known poses. Survival: share of the map's (pass A) visible features still matched correctly in pass B at the same frame. Near rut edge: feature's ground point within 3 cm of a trench edge.

Off-rut survival: the same for map features more than 0.15 m from every wheel track.

| light | pass | features per image | near a rut edge | survival vs A (mean / worst frame) | off-rut survival |
| --- | --- | --- | --- | --- | --- |
| diffuse | A: no ruts (the map) | 553 | 7.9 % | - | - |
| diffuse | control: A rendered again | 565 | 7.8 % | 52 % / 35 % | 53 % |
| diffuse | control: no ruts, B32's camera poses | 538 | 8.2 % | 35 % / 17 % | 36 % |
| diffuse | B: 3.2 cm ruts, robot in them | 1425 | 75.8 % | 16 % / 2 % | 19 % |
| diffuse | B: 3.2 cm ruts, A's camera poses | 1409 | 74.7 % | 23 % / 2 % | 26 % |
| diffuse | B: 5 cm ruts, robot in them | 1541 | 74.7 % | 11 % / 0 % | 13 % |
| low | A: no ruts (the map) | 54 | 2.6 % | - | - |
| low | control: A rendered again | 65 | 4.1 % | 32 % / 0 % | 31 % |
| low | control: no ruts, B32's camera poses | 52 | 4.5 % | 18 % / 0 % | 18 % |
| low | B: 3.2 cm ruts, robot in them | 332 | 78.4 % | 13 % / 0 % | 13 % |
| low | B: 3.2 cm ruts, A's camera poses | 311 | 76.2 % | 22 % / 0 % | 23 % |
| low | B: 5 cm ruts, robot in them | 539 | 75.1 % | 12 % / 0 % | 12 % |
