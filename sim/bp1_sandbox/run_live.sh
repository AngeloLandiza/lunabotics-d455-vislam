#!/usr/bin/env bash
# Live RViz2 view of one approach run (sandbox ray marcher + log-odds detector, frame by frame).
#   sim/bp1_sandbox/run_live.sh [live_view.py flags]   e.g.  --seed 1 --sensor d455 --dust --rate 1 --loop
# Starts rviz2 with live_view.rviz, then the publisher in this terminal (space pause, n step, r restart, q quit).
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
set +u; source /opt/ros/humble/setup.bash
rviz2 -d "$HERE/live_view.rviz" > /tmp/bp1_rviz2.log 2>&1 &
RVIZ=$!
trap 'kill $RVIZ 2>/dev/null' EXIT
# rviz2 ignores the window size in the config (it opens at ~350x330 here), so resize the window once it exists
( for _ in $(seq 40); do
    W=$(xwininfo -root -tree 2>/dev/null | grep -F 'live_view.rviz' | grep RViz | head -1 | awk '{print $1}')
    if [ -n "$W" ]; then
      python3 - "$W" <<'PY'
import ctypes, sys
x = ctypes.cdll.LoadLibrary("libX11.so.6"); x.XOpenDisplay.restype = ctypes.c_void_p
d = x.XOpenDisplay(None); w = ctypes.c_ulong(int(sys.argv[1], 16))
x.XMoveResizeWindow(ctypes.c_void_p(d), w, 60, 40, 1500, 920); x.XRaiseWindow(ctypes.c_void_p(d), w)
x.XFlush(ctypes.c_void_p(d))
PY
      break
    fi
    sleep 0.5
  done ) &
cd "$HERE" && python3 live_view.py "$@"
