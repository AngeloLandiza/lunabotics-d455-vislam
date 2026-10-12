#!/usr/bin/env bash
# NVIDIA Cosmos 3 Edge is the decision maker of a full 15-minute Lunabotics run in the Artemis
# Arena sim, on top of the arena SLAM stack (lunabotics_slam/launch/lunabotics.launch.py: fusion SLAM, terrain map,
# obstacle segmentation). Each step the model sees the rover's camera + a status report built from
# SLAM (pose, mapped obstacles, measured berm) and picks go_to / turn / drive / dig / dump / wait;
# go_to plans an A* path on the SLAM obstacle grid (sim/cosmos_agent.py).
#
#   run/sim/arena/cosmos.sh                     full run: Isaac (arena from above), RViz (SLAM map), model window
#   run/sim/arena/cosmos.sh --headless          no Isaac window and no RViz
#   run/sim/arena/cosmos.sh --seed 4            another round: new boulders/craters and start pose
#   run/sim/arena/cosmos.sh --start-heading N   start facing N/E/S/W (default: random)
#   run/sim/arena/cosmos.sh --minutes 5         shorter run
#   run/sim/arena/cosmos.sh --gt-pose           no SLAM: true pose, depth-camera hazards only
#   run/sim/arena/cosmos.sh --think-tokens 128  reason on every step (default 0: only after a refused,
#                                               stopped or timed-out action, with --trouble-tokens 128)
#   run/sim/arena/cosmos.sh --quant bf16        model precision: int8 (default, fastest), int4, bf16, nf4
#   run/sim/arena/cosmos.sh --no-viewer         no model window (camera frame, status, live reasoning)
#
# One-time setup: sim/setup_cosmos.sh (model + Python env), plus Isaac Sim (see README).
# The score card and every step (camera frame, status, reasoning, action) are saved in
# ~/.ros/cosmos_runs/<time>/.
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../../.."
ENV="${COSMOS_ENV:-$HOME/cosmos-env}"
[ -x "$ENV/bin/python" ] || { echo "[cosmos] run sim/setup_cosmos.sh first" >&2; exit 1; }
sim_args=(--world arena); seed=0; minutes=15; think=0; trouble=128; quant=int8; viewer=1; slam=1; viz=true
while [ $# -gt 0 ]; do
  case "$1" in
    --seed) seed="$2"; sim_args+=(--seed "$2"); shift ;;
    --minutes) minutes="$2"; shift ;;
    --think-tokens) think="$2"; shift ;;
    --trouble-tokens) trouble="$2"; shift ;;
    --quant) quant="$2"; shift ;;
    --no-viewer) viewer=0 ;;
    --gt-pose) slam=0 ;;
    --headless) viz=false; sim_args+=(--headless) ;;
    *) sim_args+=("$1") ;;
  esac
  shift
done

pids=()
# Each background job gets its own process group (set -m), and cleanup ends whole groups: a plain
# kill left `ros2 launch`'s nodes (rtabmap, ...) running, since background jobs ignore SIGINT.
set -m
cleanup() { for p in "${pids[@]}"; do kill -- -"$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# Model first (int4/int8 compile ~1 min the first time, then ~20 s from torch's cache), then Isaac.
"$ENV/bin/python" "$REPO/sim/cosmos_server.py" --quant "$quant" &
pids+=($!)
until curl -s -m 2 http://127.0.0.1:8000/v1/models >/dev/null; do
  kill -0 "${pids[0]}" 2>/dev/null || { echo "[cosmos] model server exited"; exit 1; }
  sleep 1
done

"$REPO/sim/start_isaac.sh" "${sim_args[@]}" &
pids+=($!)
source /opt/ros/humble/setup.bash
echo "[cosmos] waiting for Isaac Sim (/clock)..."
until timeout 5 ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock >/dev/null 2>&1; do   # no daemon: a stuck one hung this
  kill -0 "${pids[1]}" 2>/dev/null || { echo "[cosmos] Isaac Sim exited"; exit 1; }
  sleep 2
done

localization=ground_truth
if [ "$slam" = 1 ]; then
  localization=slam
  ros2 launch "$REPO/lunabotics_slam/launch/lunabotics.launch.py" sim:=true viz:="$viz" \
      database_path:="$HOME/.ros/lunabotics_cosmos.db" terrain_file:="$HOME/.ros/lunabotics_terrain_cosmos.npz" &
  pids+=($!)
fi
if [ "$viewer" = 1 ]; then
  python3 "$REPO/sim/cosmos_viewer.py" &   # what the model sees, thinks and decides, live
  pids+=($!)
fi
python3 "$REPO/sim/cosmos_agent.py" --ros-args -p use_sim_time:=true -p seed:="$seed" \
    -p minutes:="$minutes" -p localization:="$localization" \
    -p think_tokens:="$think" -p trouble_think_tokens:="$trouble"
if [ -t 0 ] && [ "$viz" = true ]; then
  read -r -p "[cosmos] run over. Press Enter to close Isaac Sim... " _ || true
fi
