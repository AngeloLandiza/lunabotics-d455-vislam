#!/usr/bin/env bash
# Render D455 stereo along the drift drives in Isaac Sim (GUI window open), then run stereo VO on them.
#   sim/odometry/run_isaac_vo.sh RESULTS_DIR [isaac_vo.py args...]
# Same environment handling as bp1_sandbox/run_isaac_capture.sh: Isaac Sim's Python must not see system ROS paths.
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
RES=$(realpath -m "$1"); shift
RAW="$HERE/isaac_out/$(basename "$RES")"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    OMNI_KIT_ACCEPT_EULA=YES "$ISAAC_PY" "$HERE/isaac_vo.py" --out "$RAW" "$@"
cd "$HERE" && env -u PYTHONPATH "$ISAAC_PY" vo_eval.py "$RAW" "$RES"
