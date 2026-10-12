#!/usr/bin/env bash
# Self-mask, calibration-error and rut cases for the team's Unitree L1 on the 2026 robot (48 arenas, seeds 3000-3047).
# Usage: ./run_selfmask.sh [out_root] [workers]     (about 1.5 h on 4 cores; the D455 cases take most of it)
set -euo pipefail
cd "$(dirname "$0")"
OUT=${1:-results_selfmask}
W=${2:-4}
N=48
run() { local name=$1; shift; [ -f "$OUT/$name/approaches.json" ] && { echo "skip $name"; return; }
        python3 approach.py $N "$OUT/$name" 3000 --robot --lidar l1 --workers "$W" --label "$name" "$@"; }
mkdir -p "$OUT"
run l1_table   --body table --sensors lidar
run l1_none    --body none  --sensors lidar
run l1_raw     --body raw   --sensors lidar
run l1_box     --body box   --sensors lidar
run l1_table_tilt1 --body table --sensors lidar --clear-only --mask-err 1
run l1_table_tilt2 --body table --sensors lidar --clear-only --mask-err 2
run l1_box_tilt1   --body box   --sensors lidar --clear-only --mask-err 1
run both_noruts    --body table --clear-only
run both_ruts1.5   --body table --clear-only --ruts 0.015
run both_ruts3.2   --body table --clear-only --ruts 0.032
run both_ruts5     --body table --clear-only --ruts 0.05
python3 speed_gate.py "$OUT/l1_table" > /dev/null
python3 speed_gate.py "$OUT/both_noruts" > /dev/null
python3 selfmask_eval.py "$OUT/eval_mask" "$OUT/l1_table" "$OUT"/l1_none "$OUT"/l1_raw "$OUT"/l1_box "$OUT"/l1_table \
    "$OUT"/l1_table_tilt1 "$OUT"/l1_table_tilt2 "$OUT"/l1_box_tilt1
python3 selfmask_eval.py "$OUT/eval_ruts" "$OUT/both_noruts" "$OUT"/both_noruts "$OUT"/both_ruts1.5 \
    "$OUT"/both_ruts3.2 "$OUT"/both_ruts5
