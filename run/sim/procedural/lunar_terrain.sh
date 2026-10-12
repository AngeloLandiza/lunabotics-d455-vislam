#!/usr/bin/env bash
# Lunabotics terrain SLAM in Isaac Sim 6: tank-drive rover with a down-pitched D455 on
# procedural lunar terrain, running the same SLAM + terrain pipeline as the real robot.
#
#   run/sim/procedural/lunar_terrain.sh               Isaac window + RViz + camera window, autonomous survey loop
#   run/sim/procedural/lunar_terrain.sh --headless    no Isaac window (saves GPU memory)
#   run/sim/procedural/lunar_terrain.sh --dig-at 62   dig a hole at sim time 62 s (just after lap 1, beside the
#                                                     loop) to watch the terrain map re-map it and report the dug volume
#   run/sim/procedural/lunar_terrain.sh --teleop      drive yourself with the keyboard instead of the survey
#   run/sim/procedural/lunar_terrain.sh --seed 3      different terrain
#
# One-time setup: Isaac Sim 6.0.1 in ~/isaacsim-env (see README).
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../../.."
sim_args=(); teleop=0
for a in "$@"; do
  case "$a" in --teleop) teleop=1 ;; *) sim_args+=("$a") ;; esac
done

pids=()
# Each background job gets its own process group (set -m), and cleanup ends whole groups: a plain
# kill left `ros2 launch`'s nodes (rtabmap, ...) running, since background jobs ignore SIGINT.
set -m
cleanup() { for p in "${pids[@]}"; do kill -- -"$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

"$REPO/sim/start_isaac.sh" "${sim_args[@]}" &
pids+=($!)

source /opt/ros/humble/setup.bash
echo "[lunar_terrain] waiting for Isaac Sim (/clock)..."
until timeout 5 ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock >/dev/null 2>&1; do   # no daemon: a stuck one hung this
  kill -0 "${pids[0]}" 2>/dev/null || { echo "[lunar_terrain] Isaac Sim exited"; exit 1; }
  sleep 2
done

ros2 launch "$REPO/lunabotics_slam/launch/lunabotics.launch.py" sim:=true \
    database_path:="$HOME/.ros/lunabotics_sim.db" terrain_file:="$HOME/.ros/lunabotics_terrain_sim.npz" &
pids+=($!)

if [ "$teleop" = 1 ]; then
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
  python3 "$REPO/sim/survey_driver.py" --ros-args -p use_sim_time:=true &
  pids+=($!)
  wait "${pids[0]}"
fi
