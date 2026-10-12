#!/usr/bin/env bash
# Tuning sweep for the "tuned Nav2" baseline: each variant below runs the same missions on tuning arenas (seeds 1000+,
# kept apart from the 3000-3099 test arenas) side by side, one ROS_DOMAIN_ID each. Pick the best variant from
# ../compare.py --merge on the outputs, then run it on the test arenas with run_nav2.sh and the same --set flags.
#
# Usage: ./tune.sh OUT_DIR [--seeds 20] [--slip nominal] [--rtf 2.0] [--jobs 3]
# The variants are a first guess from the MPPI README's tuning notes and this robot (Assumed), meant to be revised.
set -eo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[ $# -ge 1 ] || { sed -n '6p' "$0"; exit 2; }
OUT=$(realpath -m "$1"); shift
SEEDS=20 SLIP=nominal RTF=2.0 JOBS=3
while [ $# -gt 0 ]; do
  case $1 in
    --seeds) SEEDS=$2; shift 2 ;;
    --slip) SLIP=$2; shift 2 ;;
    --rtf) RTF=$2; shift 2 ;;
    --jobs) JOBS=$2; shift 2 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done

C=controller_server.FollowPath
declare -A V=(
  [default]=""
  # dump poses sit 5 cm from the berm: stop the obstacle preference sooner and weigh the final heading earlier
  [goal]="$C.CostCritic.near_goal_distance=0.5 $C.GoalAngleCritic.threshold_to_consider=1.0 $C.GoalAngleCritic.cost_weight=5.0"
  # follow the lattice path more closely (it already keeps the footprint clear)
  [track]="$C.PathAlignCritic.cost_weight=20.0 $C.PathFollowCritic.cost_weight=8.0"
  # more samples and wider turn-rate sampling (the robot turns in place a lot)
  [samples]="$C.batch_size=2000 $C.wz_std=0.6"
  # steeper cost decay so narrow gaps cost less to pass through
  [narrow]="costmaps.inflation_layer.cost_scaling_factor=6.0"
  # let the lattice planner reverse (the custom planners and MPPI both can) and search longer: in the default run some
  # dump legs got no plan within 10 s
  [reverse]="planner_server.GridBased.allow_reverse_expansion=true planner_server.GridBased.max_planning_time=8.0"
  # smaller safety margin: at 0.70 m most gaps are high cost and MPPI stops ~0.5 m short of dig spots near rocks
  # (default run, RViz). Smac and MPPI both check the full footprint, so the margin need not cover the circumscribed radius
  [inflate]="costmaps.inflation_layer.inflation_radius=0.45 costmaps.inflation_layer.cost_scaling_factor=6.0"
  # slower turning: at the goal Nav2 stops while turning at up to 1.85 rad/s, and the wheels' 0.5 m/s^2 spin-down
  # carries the body on by up to ~50 deg (11-17 deg seen in the default run with no slip)
  [slowturn]="$C.wz_max=0.9 $C.GoalAngleCritic.cost_weight=8.0"
  # all of the above together
  [combo]="costmaps.inflation_layer.inflation_radius=0.45 costmaps.inflation_layer.cost_scaling_factor=6.0 $C.wz_max=0.9 $C.GoalAngleCritic.cost_weight=8.0 planner_server.GridBased.allow_reverse_expansion=true planner_server.GridBased.max_planning_time=8.0"
  # cheaper in-place turns for the lattice planner
  [turns]="planner_server.GridBased.rotation_penalty=2.0 planner_server.GridBased.non_straight_penalty=1.0"
)

mkdir -p "$OUT"
domain=60
running=0
VIZ_PID=
for name in "${!V[@]}"; do
  sets=()
  for kv in ${V[$name]}; do sets+=(--set "$kv"); done
  echo "variant $name: ${V[$name]:-(defaults)}"
  # one RViz window at any time: the next variant to start takes it over when the variant showing it ends, so the
  # sweep is never all headless; the others run alongside without one
  if [ -n "$VIZ_PID" ] && kill -0 "$VIZ_PID" 2>/dev/null; then sets+=(--no-rviz); viz=0; else viz=1; fi
  "$HERE/run_nav2.sh" "$OUT/$name.json" --label "nav2_$name" --domain $domain --rtf "$RTF" "${sets[@]}" \
    --seeds "$SEEDS" --seed0 1000 --slip "$SLIP" >"$OUT/$name.out" 2>&1 &
  [ "$viz" = 1 ] && VIZ_PID=$!
  domain=$((domain + 1)); running=$((running + 1))
  if [ "$running" -ge "$JOBS" ]; then wait -n; running=$((running - 1)); fi
done
wait
files=$(ls "$OUT"/*.json | grep -v summary | paste -sd,)
python3 "$HERE/../compare.py" "$OUT/merged" --merge "$files"
