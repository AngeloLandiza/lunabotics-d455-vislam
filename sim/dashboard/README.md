# Sandbox dashboard

A page in your browser that shows what the computer is doing, how simulation runs are progressing, the latest
results, every parameter in `params.py`, and how the research files held up when their claims were checked.
It runs offline: Python standard library only (3.10 or newer), plain JavaScript, charts drawn as SVG by `app.js`.
Nothing is installed and nothing is fetched from the internet.

## Start it

From the repo root:

```bash
python3 sim/dashboard/server.py            # http://127.0.0.1:8765/
python3 sim/dashboard/server.py --port 9000
```

Open the printed address in a browser. The server listens on 127.0.0.1 only, so other machines cannot reach it.
Stop it with Ctrl+C. `--verbose` logs every request and prints tracebacks.

## What each section reads

| Section | Refresh | Reads |
| --- | --- | --- |
| Live | 2 s | `/api/system` (CPU load against core count, CPU busy share, RAM from `/proc/meminfo`, free disk on the repo's drive, `nvidia-smi` for GPU use, memory, temperature and power, and running simulation programs from `/proc/*/cmdline`) and `/api/runs` (runs in progress plus runs that ended in the last 15 minutes: progress bar, done/total, time left, last update, latest numbers, and a sparkline of items done over time) |
| Runs | 5 s | `/api/runs` for the table (kind, label, start, duration, status, computer); click a row for `/api/run/<id>`: its settings, a chart of every number it reported over time, and its progress lines |
| Results | 30 s | `/api/results`: `robustness.json` (a dot chart of phantom stops per 10 m at 0.75 m/s per case, and a table per case), `speed_gate.json` (chosen thresholds, the constant-speed table, the speed-gate table), `all96_summary.json`, the `meta` of every `approaches.json` / `approaches_meta.json`, and the counts in `records.json`. The source file is printed under every table and chart |
| Parameters | 60 s | `/api/params`: every entry of `params.all_tables()` with value, unit, status (Verified, Inferred, Assumed) and source; filter by status (the count tiles) or by text |
| Research | 60 s | `/api/research` (`research_index.json`): a stacked bar per research file and per check file of confirmed, partly confirmed, mismatch, not found, could not open and not checked rows, totals per step and overall, and the list of research files with titles and check notes |

Each sensor keeps one color and one marker shape everywhere: 16-beam LiDAR blue circle, D455 green square,
LiDAR or D455 (fused) orange diamond. (The matplotlib figures in the results folders draw the fused case in violet;
the dashboard uses orange because blue and violet are hard to tell apart for color-blind readers.) Status chips
always carry their word, not just a color. Light and dark mode follow the system setting.

Programs listed under Live are Python processes whose command line mentions `approach.py`, `robustness.py`,
`speed_gate.py`, `evaluate.py`, `isaac`, `live_view` or `kit` (plus a bare `kit` / `isaac-sim` executable).
Pool workers are folded into their parent; CPU time and memory include the workers.

## Reporting progress from a run: `sim/bp1_sandbox/telemetry.py`

```python
from telemetry import Telemetry

tel = Telemetry(kind="approach", total=a.n, args=a, label="floor waves 4 cm")   # args: dict or argparse Namespace
with Pool(a.workers) as p:
    for k, r in enumerate(p.imap(trial, seeds)):
        res.append(r)
        tel.progress(k + 1, last_seed=r["seed"])     # in the parent, as results arrive
tel.end(seconds=time.time() - t0)                     # or tel.error("what went wrong")
```

`with Telemetry(...) as tel:` also works: it calls `end()` on a normal exit and `error()` on an exception.
If the program exits without either, an `error` line saying so is written at exit.

- File: `<dir>/<run_id>.jsonl`, one JSON object per line. `<dir>` is `$DASH_TELEMETRY_DIR` if set, else
  `<repo>/sim/telemetry/` (created when missing). The run id is `<kind>-<UTC %Y%m%d-%H%M%S>-<pid>`.
- Fields: `t` (epoch seconds), `run`, `kind`, `label`, `event` (`start`, `progress`, `end`, `error`), `done`,
  `total`, `elapsed_s`, `eta_s` (from the average pace so far), `rate_per_s`, `host`, `pid`, `args` (start line
  only), `metrics` (numbers or strings up to 200 characters; NaN and infinity become null), `msg`.
- It never stops a run: every file operation is guarded; if the directory cannot be written it prints one warning
  on stderr and does nothing after that.
- Only the process that created the object writes. A copy inherited by a forked Pool worker stays silent, so call
  `progress()` in the parent.

Run status on the dashboard: Running; Finished (an `end` line); Failed (an `error` line); No updates (no line for
10 minutes and no end line); Stopped without finishing (same computer, the process is gone, no end line).

## API

All endpoints return JSON. Any failure returns `{"error": "..."}` with HTTP 500 instead of stopping the server.

| Endpoint | Returns |
| --- | --- |
| `/api/system` | `hostname`, `cpu_count`, `load` (1/5/15 min), `cpu_busy` (share since the previous call), `memory` (`total`, `available`, `used`, swap, bytes), `disk` (`total`, `used`, `free` for the repo's drive), `gpu` (list of `name`, `util_pct`, `mem_used_mib`, `mem_total_mib`, `temp_c`, `power_w`, or null when `nvidia-smi` is missing, fails or takes over 2 s), `processes` (`pid`, `cmd`, `cpu_s`, `age_s`, `rss`, `workers`, `workers_cpu_s`, `cpu_pct`) |
| `/api/runs` | `dir`, `runs`: one object per run with its first and last lines merged: `run`, `kind`, `label`, `host`, `pid`, `args`, `status`, `done`, `total`, `progress`, `eta_s`, `eta_now_s`, `started`, `updated`, `age_s`, `duration_s`, `metrics` (latest value of each), `msg`, `spark` (up to 80 `[t, done]` points) |
| `/api/run/<run_id>` | `summary` and every line of that run (404 if unknown) |
| `/api/results` | `items`: `path` (relative to the repo), `type`, `mtime`, `size`, `content`. Roots: folders named `results*` in `sim/bp1_sandbox` and `sim/bp1_sandbox/summaries`, searched 3 levels deep. `records.json` keeps only `n_trials`, `seed0`, `seconds`; `approaches.json` only its `meta` (read from the start of the file); results are cached by path and modification time |
| `/api/params` | `tables` (each entry: `name`, `value`, `unit`, `status`, `source`), `counts` per status, `total`. `params.py` is re-imported when it changes. Plain-dict tables (such as `LIDAR_PROFILES`) are handled if `all_tables()` ever includes them |
| `/api/research` | the contents of `research_index.json`, or an empty structure if it is missing |
| `/api/planner` | `items`: planner benchmark results, one item per (file, planner, slip) combination. Read fresh on every request from `sim/planner/results/compare_v*/summary.json` (summary statistics) and `sim/planner/results/nav2/*.json` (live nav2 runs); skips files with JSON decode errors. Each item: `path`, `mtime`, `type` (summary or nav2), `planner`, `slip`, `missions`, `complete`, `legs_with_contact`, `mission_time_median`, `mission_time_p95`, `plan_s_median`, `plan_s_p95`, `dump_on_target` (null for nav2 on some fields), `runs` (array with seed/slip/complete/time and legs detail for nav2 files) |

## Refreshing the copies in the repo

Research index (reads the research folder, never writes there):

```bash
python3 sim/dashboard/make_research_index.py                  # default --root /mnt/project-files/research
python3 sim/dashboard/make_research_index.py --root path/to/research
```

It reads `step*/NN-*.md` (title = first `# ` heading, size, and the first line when it is a `> Check status` note)
and the `## Result counts` section of `step*/verify/verify-*.md`. Columns are matched by name, case-insensitive
(`Rows`, `Rows checked`, `Claims`, `Total rows` all count as rows), and a check file that writes its counts as
`File 19 (90 claim rows): Confirmed 71; Mismatch 5; ...` is read from that text. Rows without a rows column get the
sum of their categories. It prints a note for anything it could not read.

Result summaries (copies small files from the cloud results into `sim/bp1_sandbox/summaries/<same path>/`):

```bash
python3 sim/dashboard/collect_summaries.py --dry-run          # list what would be copied
python3 sim/dashboard/collect_summaries.py                    # default --src /mnt/project-files/step2/bp1_sandbox
```

It copies `robustness.json/.md`, `speed_gate.json/.md`, `all96_summary.json`, `tables.md` and PNGs under 400 KB,
skips anything over 1 MB, and writes only the `meta` of each `approaches.json` as `approaches_meta.json`. Commit
the result and `git pull` on the laptop to see the cloud results there.

## .gitignore

- Add `sim/telemetry/` to the root `.gitignore` (progress files are per machine).
- `sim/bp1_sandbox/.gitignore` has unanchored patterns (`results/`, `results_v*/`, `results_approach*/`,
  `results_rob/`) that also match the folders inside `sim/bp1_sandbox/summaries/`, so the copies are ignored as it
  stands. Append this line to `sim/bp1_sandbox/.gitignore` to keep ignoring the real results folders but track the
  copies:

  ```
  !/summaries/**/
  ```

  (Or `git add -f sim/bp1_sandbox/summaries` once, and again after new files appear.)
