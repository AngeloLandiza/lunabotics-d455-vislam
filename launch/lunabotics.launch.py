# Lunabotics terrain SLAM: fusion localization + semi-dynamic terrain map.
#
#   d455_fusion.launch.py -> visual-inertial tracking (projector-off IR) + ICP-refined,
#                            gravity-constrained pose graph on dense depth (projector-on)
#   terrain_mapper.py     -> 2.5D height map in the map frame that re-maps dug/dumped
#                            areas and reports dug (cut) and dumped (fill) volume
#   obstacle_segmenter.py -> boulders / craters / dug pits / piles on that map, as RViz
#                            markers (/terrain/obstacles) and an occupancy grid
#                            (/terrain/obstacle_grid) for driving and path planning
#
# Competition workflow (prior-map anchoring, the arena analogue of DEM anchoring):
#   1. ./run_lunabotics.sh                      survey: map the arena + terrain baseline
#   2. ./run_lunabotics.sh localization:=true   runs: localize in that map, load the saved
#                                               terrain, report volume vs. the survey

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    passthrough = ['viz', 'localization', 'new_map', 'database_path', 'config', 'rtabmap_args', 'sim']

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(REPO, 'launch', 'd455_fusion.launch.py')),
        launch_arguments=[(a, LaunchConfiguration(a)) for a in passthrough] + [
            ('rviz_config', os.path.join(REPO, 'rviz', 'lunabotics.rviz'))],
    )

    terrain = ExecuteProcess(
        cmd=['python3', os.path.join(REPO, 'scripts', 'terrain_mapper.py'), '--ros-args',
             '--params-file', LaunchConfiguration('terrain_config'),
             '-p', ['load:=', LaunchConfiguration('load_terrain')],
             '-p', ['use_sim_time:=', LaunchConfiguration('sim')],
             '-p', ['save_path:=', LaunchConfiguration('terrain_file')]],
        output='screen',
    )

    obstacles = ExecuteProcess(
        cmd=['python3', os.path.join(REPO, 'scripts', 'obstacle_segmenter.py'), '--ros-args',
             '--params-file', LaunchConfiguration('terrain_config'),
             '-p', ['use_sim_time:=', LaunchConfiguration('sim')]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in the saved arena map instead of mapping.'),
        DeclareLaunchArgument('new_map', default_value='true'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/lunabotics.db')),
        DeclareLaunchArgument('config',
                              default_value=os.path.join(REPO, 'config', 'd455_fusion.yaml')),
        DeclareLaunchArgument('terrain_config',
                              default_value=os.path.join(REPO, 'config', 'lunabotics_terrain.yaml')),
        DeclareLaunchArgument('terrain_file',
                              default_value=os.path.expanduser('~/.ros/lunabotics_terrain.npz'),
                              description='Terrain map + baseline file (saved on exit, loaded with load_terrain).'),
        DeclareLaunchArgument('load_terrain', default_value=LaunchConfiguration('localization'),
                              description='Start from the saved terrain/baseline (default: in localization).'),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Use Isaac Sim (./run_sim.sh) instead of the real D455.'),
        slam,
        terrain,
        obstacles,
    ])
