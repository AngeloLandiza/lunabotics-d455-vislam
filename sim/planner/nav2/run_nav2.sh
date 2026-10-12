#!/usr/bin/env bash
# Nav2 baseline for the planner head-to-head (see ../README.md): starts Nav2 (Smac State Lattice + MPPI) on the shared
# 2D arena and runs the same missions as compare.py through sim_bridge.py. Needs ROS 2 Humble with nav2_bringup,
# nav2_smac_planner and nav2_mppi_controller.
#
# Usage: ./run_nav2.sh OUT.json [--label nav2_default] [--params nav2_params.yaml] [--domain 42] [--rtf 2.0]
#                      [--set NODE.KEY.SUBKEY=VALUE ...] [--seeds 100] [--seed0 3000] [--slip nominal] [--settle 2.5]
#   --set overrides one parameter of the params file, e.g. --set controller_server.FollowPath.vx_std=0.3;
#   NODE "costmaps" sets both costmaps, e.g. --set costmaps.inflation_layer.inflation_radius=0.55
#   e.g. ./run_nav2.sh ../results/nav2/nav2_default.json --slip nominal --seeds 2      (smoke test)
# Runs resume: missions already in OUT.json are skipped. Nav2's log goes to OUT.nav2.log.
#
# --rtf R runs the sim at R sim seconds per wall second. Nav2 Humble times the controller loop (rclcpp::WallRate) and
# the velocity smoother (create_wall_timer, accel / smoothing_frequency per tick) on the wall clock (Verified in the
# humble branch sources, 2026-10-12), so this script multiplies those two rates and the smoother's acceleration limits
# by R, which keeps the params file's values true in sim time. The bridge logs the command rate in sim time for every
# leg: about 20 Hz means the scaling works. Not scaled: the default BT's 1 Hz replanning (wall clock too), so Nav2
# replans every R sim seconds; the map is static, so this should matter little (Inferred).
# Several runs can go side by side with different --domain values (one ROS_DOMAIN_ID each).
set -eo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[ $# -ge 1 ] || { sed -n '6,11p' "$0"; exit 2; }
OUT=$(realpath -m "$1"); shift
LABEL=nav2_default PARAMS=$HERE/nav2_params.yaml DOMAIN=${ROS_DOMAIN_ID:-42} RTF=2.0 PASS=() SETS=()
while [ $# -gt 0 ]; do
  case $1 in
    --label) LABEL=$2; shift 2 ;;
    --params) PARAMS=$(realpath "$2"); shift 2 ;;
    --domain) DOMAIN=$2; shift 2 ;;
    --rtf) RTF=$2; shift 2 ;;
    --set) SETS+=("$2"); shift 2 ;;
    *) PASS+=("$1"); shift ;;
  esac
done

source /opt/ros/humble/setup.bash
for p in nav2_bringup nav2_smac_planner nav2_mppi_controller nav2_msgs; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || { echo "ROS package $p is not installed"; exit 1; }
done

# 5 cm differential-drive lattice primitives shipped with nav2_smac_planner (the costmap is 5 cm too)
SHARE=$(ros2 pkg prefix nav2_smac_planner)/share/nav2_smac_planner
LATTICES=$(find "$SHARE" -name output.json -path '*5cm*' -path '*diff*' | sort)
LATTICE=$(echo "$LATTICES" | grep -m1 '0.5m' || true)
[ -n "$LATTICE" ] || LATTICE=$(echo "$LATTICES" | head -n1)
[ -n "$LATTICE" ] || { echo "no 5 cm diff-drive lattice primitives under $SHARE"; exit 1; }
echo "lattice: $LATTICE"

RUN=$(mktemp -d)
NAV=
cleanup() {
  [ -n "$NAV" ] && kill -INT "$NAV" 2>/dev/null && wait "$NAV" 2>/dev/null
  rm -rf "$RUN"
}
trap cleanup EXIT

python3 - "$PARAMS" "$RUN/params.yaml" "$LATTICE" "$RTF" "${SETS[@]}" <<'EOF'
import sys
import yaml
src, dst, lattice, rtf = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
p = yaml.safe_load(open(src))
for item in sys.argv[5:]:
    key, val = item.split("=", 1)
    node, *path = key.split(".")
    for n in (["local_costmap", "global_costmap"] if node == "costmaps" else [node]):
        d = p[n][n]["ros__parameters"] if n in ("local_costmap", "global_costmap") else p[n]["ros__parameters"]
        for k in path[:-1]:
            d = d[k]
        if path[-1] not in d:
            sys.exit(f"--set {key}: no such parameter in the params file")
        d[path[-1]] = yaml.safe_load(val)
p["planner_server"]["ros__parameters"]["GridBased"]["lattice_filepath"] = lattice
p["controller_server"]["ros__parameters"]["controller_frequency"] *= rtf
s = p["velocity_smoother"]["ros__parameters"]
s["smoothing_frequency"] *= rtf
s["max_accel"] = [a * rtf for a in s["max_accel"]]
s["max_decel"] = [a * rtf for a in s["max_decel"]]
yaml.safe_dump(p, open(dst, "w"))
EOF

export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
mkdir -p "$(dirname "$OUT")"
LOG=${OUT%.json}.nav2.log
ros2 launch nav2_bringup navigation_launch.py params_file:="$RUN/params.yaml" use_sim_time:=True autostart:=True \
  >"$LOG" 2>&1 &
NAV=$!
echo "Nav2 started (log $LOG), domain $DOMAIN, rtf $RTF"
python3 "$HERE/sim_bridge.py" "$OUT" --label "$LABEL" --rtf "$RTF" "${PASS[@]}"
