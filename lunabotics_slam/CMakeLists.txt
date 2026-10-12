cmake_minimum_required(VERSION 3.8)
project(lunabotics_slam NONE)

find_package(ament_cmake REQUIRED)

# Everything keeps the source layout under share/lunabotics_slam/, because the launch files find
# config/, rviz/ and scripts/ next to their own directory. That is what lets the same launch
# files run straight from the repo (run/*.sh) and from an installed workspace (ros2 launch).
install(DIRECTORY launch config rviz udev DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/emitter_splitter.py
  scripts/obstacle_segmenter.py
  scripts/terrain_mapper.py
  scripts/vislam_viewer.py
  scripts/arena_localizer.py
  scripts/hazard_detector.py
  scripts/mission_runner.py
  scripts/start_tags.py
  DESTINATION share/${PROJECT_NAME}/scripts)
# Also as `ros2 run lunabotics_slam <script>.py`
install(PROGRAMS
  scripts/emitter_splitter.py
  scripts/obstacle_segmenter.py
  scripts/terrain_mapper.py
  scripts/vislam_viewer.py
  scripts/arena_localizer.py
  scripts/hazard_detector.py
  scripts/mission_runner.py
  scripts/start_tags.py
  DESTINATION lib/${PROJECT_NAME})

# Modules the autonomy nodes import (they also need the repo's sim/ folder: set LUNABOTICS_REPO when installed)
install(FILES scripts/autonomy_core.py scripts/autonomy_ros.py DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/autonomy_core.py scripts/autonomy_ros.py DESTINATION lib/${PROJECT_NAME})

ament_package()
