# Isaac vs sandbox over 10 sensor-noise streams (seed 1)

Same arena, path and thresholds as compare.md; each stream gives both sources the same noise draws per frame.

| case | missed at 1.0 m/s (sum) sandbox / Isaac | phantom stops per 10 m at 1.0 m/s, mean sandbox / Isaac | streams with any phantom stop sandbox / Isaac | confirmed false-alarm frames, mean sandbox / Isaac |
| --- | --- | --- | --- | --- |
| spin16 clear | 0 / 0 | 0.00 / 0.00 | 0 / 0 | 0.0 / 0.0 |
| spin16 dust | 0 / 0 | 0.25 / 0.00 | 1 / 0 | 35.7 / 31.4 |
| d455 clear | 0 / 0 | 0.00 / 0.00 | 0 / 0 | 0.0 / 0.0 |
| d455 dust | 0 / 0 | 0.00 / 0.00 | 0 / 0 | 0.0 / 0.0 |

First confirmation frame per path hazard, per stream (sandbox | Isaac):

- spin16 clear: [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]; [1, 0, 0] | [1, 0, 0]
- spin16 dust: [1, 0, 0] | [2, 0, 0]; [2, 0, 0] | [1, 0, 1]; [1, 0, 0] | [2, 0, 0]; [1, 0, 0] | [1, 0, 0]; [2, 0, 0] | [1, 0, 0]; [2, 0, 0] | [2, 0, 0]; [1, 0, 0] | [2, 0, 0]; [1, 0, 1] | [1, 0, 0]; [2, 0, 0] | [1, 0, 1]; [2, 0, 0] | [1, 0, 1]
- d455 clear: [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]; [0, 0, 0] | [0, 0, 0]
- d455 dust: [1, 0, 1] | [1, 0, 1]; [1, 0, 2] | [1, 0, 2]; [1, 0, 1] | [1, 0, 2]; [1, 0, 1] | [1, 0, 1]; [1, 0, 3] | [1, 0, 3]; [1, 0, 1] | [1, 0, 2]; [1, 0, 2] | [1, 0, 2]; [1, 0, 2] | [1, 0, 2]; [1, 0, 2] | [1, 0, 3]; [1, 0, 1] | [1, 0, 2]
