#!/usr/bin/env bash
# The Lunabotics Artemis Arena in Isaac Sim (2027 guidebook) with the full SLAM + terrain
# pipeline and the dig tool: edit the regolith with the mouse and watch SLAM re-map it.
#
#   ./run_arena.sh                   arena + SLAM + dig tool; drive with W A S D in the dig tool window
#   ./run_arena.sh --survey          the rover drives survey loops on its own instead
#   ./run_arena.sh --eval --headless check SLAM against ground truth: survey lap, dig + dump,
#                                    second lap, then pose / map / volume errors (sim/arena_eval.py)
#   ./run_arena.sh --seed 4          another round: new boulders/craters and start pose
#   ./run_arena.sh --start-heading N start facing N/E/S/W (default: random, like the direction wheel)
#   ./run_arena.sh --headless        no Isaac window
#
# Dig tool window: hold W A S D to drive (tank drive: W/S forward/back, A/D pivot, W+A/W+D
# arc; 1 2 3 speed), left-drag digs, right-drag dumps, wheel = brush size, E cycles the SLAM
# view (elevation / change / error vs. truth), R re-aligns, Q quits.
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")"
sim_args=(--world arena); mode=teleop; seed=0
while [ $# -gt 0 ]; do
  case "$1" in
    --survey) mode=survey ;;
    --eval) mode=eval ;;
    --teleop) mode=teleop ;;
    --seed) seed="$2"; sim_args+=(--seed "$2"); shift ;;
    *) sim_args+=("$1") ;;
  esac
  shift
done

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

"$REPO/sim/start_isaac.sh" "${sim_args[@]}" &
pids+=($!)

source /opt/ros/humble/setup.bash
echo "[run_arena] waiting for Isaac Sim (/clock)..."
until timeout 3 ros2 topic echo --once /clock >/dev/null 2>&1; do
  kill -0 "${pids[0]}" 2>/dev/null || { echo "[run_arena] Isaac Sim exited"; exit 1; }
  sleep 2
done

ros2 launch "$REPO/launch/lunabotics.launch.py" sim:=true \
    database_path:="$HOME/.ros/lunabotics_arena.db" terrain_file:="$HOME/.ros/lunabotics_terrain_arena.npz" &
pids+=($!)
if [ "$mode" = eval ]; then
  python3 "$REPO/sim/arena_eval.py" --ros-args -p use_sim_time:=true -p seed:="$seed"
  exit
fi
dig_tool=(python3 "$REPO/sim/dig_tool.py" --ros-args -p use_sim_time:=true -p seed:="$seed")
if [ "$mode" = teleop ]; then
  "${dig_tool[@]}" -p drive:=true                  # closing the window ends the run
else
  "${dig_tool[@]}" &
  pids+=($!)
  python3 "$REPO/sim/arena_driver.py" --ros-args -p use_sim_time:=true -p seed:="$seed" &
  pids+=($!)
  wait "${pids[0]}"
fi
