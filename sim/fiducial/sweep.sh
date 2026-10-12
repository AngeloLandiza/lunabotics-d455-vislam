#!/usr/bin/env bash
# Sweep of the start-zone tag test over the rear camera's pitch and field of view and the tag height.
# Usage: ./sweep.sh OUT_DIR [TRIALS]
set -e
OUT=$1; N=${2:-2000}
cd "$(dirname "$0")"
for hfov in 1.2 2.0; do for pitch in 15 0 -15; do for h in 0.3 0.6 0.9; do
  d="$OUT/hfov${hfov}_pitch${pitch}_h${h}"
  echo "== hfov $hfov rad, pitch $pitch deg, tag height $h m"
  python3 tag_start.py "$d" --trials "$N" --hfov "$hfov" --cam-pitch "$pitch" --tag-height "$h" | grep -E "^(one|two|four)"
done; done; done
