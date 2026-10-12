#!/usr/bin/env bash
# Render the start-zone tags in Isaac Sim (GUI window open), then detect and score them.
#   sim/fiducial/run_isaac_tags.sh RESULTS_DIR [isaac_tags.py args...]
# Same environment handling as bp1_sandbox/run_isaac_capture.sh: Isaac Sim's Python must not see system ROS paths.
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
RES=$(realpath -m "$1"); shift
RAW="$HERE/isaac_out/$(basename "$RES")"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    OMNI_KIT_ACCEPT_EULA=YES "$ISAAC_PY" "$HERE/isaac_tags.py" --out "$RAW" "$@"
# Detection runs on Isaac's Python too: its OpenCV 4.13 decodes tag36h11, the system OpenCV 4.5.4 aruco rejected the
# same rendered tags (2026-10-12)
cd "$HERE" && env -u PYTHONPATH "$ISAAC_PY" isaac_tags_eval.py "$RAW" "$RES"
