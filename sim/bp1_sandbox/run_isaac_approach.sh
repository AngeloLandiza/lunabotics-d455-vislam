#!/usr/bin/env bash
# Approach run on Isaac Sim hits: export the arena to USD if needed, capture every approach pose in Isaac Sim
# (headless), then compare with the sandbox run (isaac_approach.py). Watch it with: ./run_live.sh --source isaac
#   sim/bp1_sandbox/run_isaac_approach.sh [--seed N] [--noise K] [--headless]
# K noise streams for the variance study (default 0). The Isaac Sim window opens on the display unless --headless.
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
SEED=1; NOISE=0; GUI=--gui
while [ $# -gt 0 ]; do case "$1" in --seed) SEED=$2; shift 2;; --noise) NOISE=$2; shift 2;; --headless) GUI=; shift;; *) echo "unknown $1"; exit 2;; esac; done
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
ISAAC_ENV=(env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH OMNI_KIT_ACCEPT_EULA=YES)
cd "$HERE"
USD="usd/arena_seed$SEED.usda"
[ -f "$USD" ] || "${ISAAC_ENV[@]}" "$ISAAC_PY" isaac_bridge.py "$SEED" "$USD"     # Isaac's Python has pxr
"${ISAAC_ENV[@]}" "$ISAAC_PY" isaac_capture.py $GUI --usd "$USD" --poses "$(python3 isaac_approach.py poses "$SEED")" \
    --out "isaac_out/approach_seed$SEED.npz"
python3 isaac_approach.py "isaac_out/approach_seed$SEED.npz" "$SEED" "$NOISE"
