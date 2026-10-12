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
  DESTINATION share/${PROJECT_NAME}/scripts)
# Also as `ros2 run lunabotics_slam <script>.py`
install(PROGRAMS
  scripts/emitter_splitter.py
  scripts/obstacle_segmenter.py
  scripts/terrain_mapper.py
  scripts/vislam_viewer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
