#!/usr/bin/env bash
# The Lunabotics Artemis Arena in Isaac Sim (2027 guidebook) with the full SLAM + terrain
# pipeline and the dig tool: edit the regolith with the mouse and watch SLAM re-map it.
#
#   run/sim/arena/arena.sh                   arena + SLAM + dig tool; drive with W A S D in the dig tool window
#   run/sim/arena/arena.sh --survey          the rover drives survey loops on its own instead
#   run/sim/arena/arena.sh --eval --headless check SLAM against ground truth: survey lap, dig + dump,
#                                            second lap, then pose / map / volume errors (sim/arena_eval.py)
#   run/sim/arena/arena.sh --seed 4          another round: new boulders/craters and start pose
#   run/sim/arena/arena.sh --start-heading N start facing N/E/S/W (default: random, like the direction wheel)
#   run/sim/arena/arena.sh --headless        no Isaac window
#   run/sim/arena/arena.sh --robot jackal    the Clearpath Jackal instead of the EDT Lunabotics rover (team URDF)
#   run/sim/arena/arena.sh --realism off     ideal sensors and rigid ground (default nominal: BP-1 sensor noise,
#                                            slip, ruts and dust from sim/realism.py; heavy: worse)
#   run/sim/arena/arena.sh --earlier-trips 6 start on a used bed with the ruts of 6 earlier drives
#
# Dig tool window: hold W A S D to drive (tank drive: W/S forward/back, A/D pivot, W+A/W+D
# arc; 1 2 3 speed), left-drag digs, right-drag dumps, wheel = brush size, E cycles the SLAM
# view (elevation / change / error vs. truth), R re-aligns, Q quits.
set -eo pipefail
REPO="$(dirname "$(readlink -f "$0")")/../../.."
sim_args=(--world arena); mode=teleop; seed=0; realism=nominal
while [ $# -gt 0 ]; do
  case "$1" in
    --survey) mode=survey ;;
    --eval) mode=eval ;;
    --teleop) mode=teleop ;;
    --seed) seed="$2"; sim_args+=(--seed "$2"); shift ;;
    --realism) realism="$2"; sim_args+=(--realism "$2"); shift ;;
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

"$REPO/sim/start_isaac.sh" "${sim_args[@]}" &
pids+=($!)

source /opt/ros/humble/setup.bash
echo "[arena] waiting for Isaac Sim (/clock)..."
until timeout 5 ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock >/dev/null 2>&1; do   # no daemon: a stuck one hung this
  kill -0 "${pids[0]}" 2>/dev/null || { echo "[arena] Isaac Sim exited"; exit 1; }
  sleep 2
done

if [ "$realism" != off ]; then        # IMU noise: the sim publishes the ideal IMU on /sim/imu_ideal
  python3 "$REPO/sim/imu_realism_relay.py" --ros-args -p level:="$realism" -p seed:="$seed" &
  pids+=($!)
fi
ros2 launch "$REPO/lunabotics_slam/launch/lunabotics.launch.py" sim:=true \
    database_path:="$HOME/.ros/lunabotics_arena.db" terrain_file:="$HOME/.ros/lunabotics_terrain_arena.npz" &
pids+=($!)
if [ "$mode" = eval ]; then
  python3 "$REPO/sim/arena_eval.py" --ros-args -p use_sim_time:=true -p seed:="$seed"
  exit
fi
dig_tool=(python3 "$REPO/sim/dig_tool.py" --ros-args -p use_sim_time:=true -p seed:="$seed")
if [ "$mode" = teleop ]; then
  echo "[arena] 'Arena Dig Tool' window: click it, hold W A S D to drive, mouse to dig, Q to quit"
  "${dig_tool[@]}" -p drive:=true                  # closing the window ends the run
else
  "${dig_tool[@]}" &
  pids+=($!)
  python3 "$REPO/sim/arena_driver.py" --ros-args -p use_sim_time:=true -p seed:="$seed" &
  pids+=($!)
  wait "${pids[0]}"
fi
