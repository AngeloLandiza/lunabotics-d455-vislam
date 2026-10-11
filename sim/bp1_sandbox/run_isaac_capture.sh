#!/usr/bin/env bash
# Capture ideal RTX LiDAR + depth in Isaac Sim (headless), then validate and score with the BP-1 layer.
#   sim/bp1_sandbox/run_isaac_capture.sh [isaac_capture.py args...]
# Same environment handling as sim/start_isaac.sh: Isaac Sim's Python 3.12 must not see system ROS paths.
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
OUT="$HERE/isaac_out/capture.npz"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    OMNI_KIT_ACCEPT_EULA=YES "$ISAAC_PY" "$HERE/isaac_capture.py" --out "$OUT" "$@"
# Validation needs numpy + scipy only, so it runs on the system python
cd "$HERE" && python3 isaac_validate.py "$OUT"
