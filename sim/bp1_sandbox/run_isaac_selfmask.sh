#!/usr/bin/env bash
# Robot self-occlusion capture in Isaac Sim (window on the display unless --headless): isaac_out/selfmask.npz/.json
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
cd "$HERE" && env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    OMNI_KIT_ACCEPT_EULA=YES "$ISAAC_PY" isaac_selfmask.py "$@"
