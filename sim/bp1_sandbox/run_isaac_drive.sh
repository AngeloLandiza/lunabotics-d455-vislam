#!/usr/bin/env bash
# Team robot drives an approach path in Isaac Sim, pressing ruts into the arena (window on the display unless
# --headless). Flags go to isaac_drive_ruts.py, e.g. --seed 1 --rut-depth 0.05 --earlier-trips 6
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
cd "$HERE" && env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    OMNI_KIT_ACCEPT_EULA=YES "$ISAAC_PY" isaac_drive_ruts.py "$@"
