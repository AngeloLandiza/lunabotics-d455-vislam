#!/usr/bin/env bash
# Start the Isaac Sim lunar scene (args go to sim/isaac_lunar_sim.py). Isaac Sim uses its
# own bundled ROS 2 Humble libraries (Python 3.12), so system ROS is stripped from its env.
set -eo pipefail
REPO="$(dirname "$(dirname "$(readlink -f "$0")")")"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim-env/bin/python}"
SITE="$(env -u PYTHONPATH "$ISAAC_PY" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_LOCALHOST_ONLY \
    OMNI_KIT_ACCEPT_EULA=YES ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LD_LIBRARY_PATH="$SITE/isaacsim/exts/isaacsim.ros2.core/humble/lib" \
    "$ISAAC_PY" "$REPO/sim/isaac_lunar_sim.py" "$@"
