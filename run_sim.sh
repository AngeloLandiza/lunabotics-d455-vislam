#!/usr/bin/env bash
# Lunabotics terrain SLAM in Isaac Sim 6: tank-drive rover with a down-pitched D455 on
# procedural lunar terrain, running the same SLAM + terrain pipeline as the real robot.
#
#   ./run_sim.sh                 Isaac window + RViz + camera window, autonomous survey loop
#   ./run_sim.sh --headless      no Isaac window (saves GPU memory)
#   ./run_sim.sh --dig-at 62     dig a hole at sim time 62 s (just after lap 1, beside the loop) to watch
#                                the terrain map re-map it and report the dug volume
#   ./run_sim.sh --teleop        drive yourself with the keyboard instead of the survey
#   ./run_sim.sh --seed 3        different terrain
#
# One-time setup: Isaac Sim 6.0.1 in ~/isaacsim-env (see README).
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")"
sim_args=(); teleop=0
for a in "$@"; do
  case "$a" in --teleop) teleop=1 ;; *) sim_args+=("$a") ;; esac
done

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

"$REPO/sim/start_isaac.sh" "${sim_args[@]}" &
pids+=($!)

source /opt/ros/humble/setup.bash
echo "[run_sim] waiting for Isaac Sim (/clock)..."
until timeout 3 ros2 topic echo --once /clock >/dev/null 2>&1; do
  kill -0 "${pids[0]}" 2>/dev/null || { echo "[run_sim] Isaac Sim exited"; exit 1; }
  sleep 2
done

ros2 launch "$REPO/launch/lunabotics.launch.py" sim:=true \
    database_path:="$HOME/.ros/lunabotics_sim.db" terrain_file:="$HOME/.ros/lunabotics_terrain_sim.npz" &
pids+=($!)

if [ "$teleop" = 1 ]; then
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
  python3 "$REPO/sim/survey_driver.py" --ros-args -p use_sim_time:=true &
  pids+=($!)
  wait "${pids[0]}"
fi
