# Stereo visual-inertial SLAM for an Intel RealSense D455.
#
# Pipeline (all stock ROS 2 Humble packages, no source builds):
#   realsense2_camera     -> left/right IR images (emitter off) + united gyro/accel IMU
#   imu_filter_madgwick   -> fuses gyro+accel into an orientation quaternion (/imu/data)
#   rtabmap_odom          -> stereo odometry, IMU used for motion prior + gravity alignment
#   rtabmap_slam          -> graph SLAM with loop closure + IMU gravity constraints
#   rviz2 + vislam_viewer -> map/trajectory window + camera/landmarks window
#
# All node settings live in config/d455_vislam.yaml; this file only picks modes
# and wires topics.
#
# Usage:
#   ros2 launch launch/d455_vislam.launch.py
#   ros2 launch launch/d455_vislam.launch.py viz:=false                 # headless (robot)
#   ros2 launch launch/d455_vislam.launch.py rtabmap_viz:=true          # stock RTAB-Map GUI too
#   ros2 launch launch/d455_vislam.launch.py localization:=true         # reuse saved map
#   ros2 launch launch/d455_vislam.launch.py database_path:=~/arena.db  # custom map file
#   ros2 launch launch/d455_vislam.launch.py new_map:=false              # extend saved map
#   ros2 launch launch/d455_vislam.launch.py dynamic_map:=true          # map forgets dug-out terrain
#   ros2 launch launch/d455_vislam.launch.py config:=config/my_room.yaml

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    localization = LaunchConfiguration('localization')
    config = LaunchConfiguration('config')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    dynamic_map = LaunchConfiguration('dynamic_map')

    def if_dynamic(on, off):
        return PythonExpression(["'", on, "' if '", dynamic_map, "'.lower() == 'true' else '", off, "'"])

    # Second rtabmap params file: the dynamic-map overlay, or the main config again (no-op).
    map_overlay = if_dynamic(os.path.join(REPO, 'config', 'dynamic_map.yaml'), config)
    database = {'database_path': LaunchConfiguration('database_path')}

    remappings = [
        ('imu', '/imu/data'),
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
    ]

    camera = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='', name='camera', output='screen', emulate_tty=True,
        parameters=[config],
    )

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[config],
        remappings=[('imu/data_raw', '/camera/imu')],
    )

    odometry = Node(
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=[config],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(localization),
        parameters=[config, map_overlay, database],
        # Fresh map per run by default: appending a new session to an existing
        # database has crashed rtabmap (Memory::addLink weight assertion).
        arguments=[PythonExpression(["'--delete_db_on_start' if '",
                                     LaunchConfiguration('new_map'),
                                     "'.lower() == 'true' else ''"]),
                   rtabmap_args],
        remappings=remappings,
    )

    slam_localization = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=IfCondition(localization),
        parameters=[config, map_overlay, database, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='vislam_map', output='log',
        condition=IfCondition(viz),
        arguments=['-d', os.path.join(REPO, 'rviz', 'vislam_map.rviz')],
        # Static map: every keyframe's cloud stitched together. Dynamic map: the
        # ray-traced OctoMap, where cleared voxels disappear.
        remappings=[('/vislam/map_cloud', if_dynamic('/octomap_occupied_space', '/cloud_map'))],
    )

    viewer = ExecuteProcess(
        condition=IfCondition(viz),
        cmd=['python3', os.path.join(REPO, 'scripts', 'vislam_viewer.py')],
        additional_env={'GDK_BACKEND': 'x11'},  # same XWayland path as RViz
        output='screen',
    )

    rtabmap_gui = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        parameters=[config],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('config',
                              default_value=os.path.join(REPO, 'config', 'd455_vislam.yaml'),
                              description='Parameter file with all camera, IMU, odometry and SLAM settings.'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open the map (RViz) and camera/landmarks windows.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Also open the stock rtabmap_viz GUI.'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Mapping mode: wipe database_path and start a fresh map '
                                          '(false = extend the existing map).'),
        DeclareLaunchArgument('dynamic_map', default_value='false',
                              description='Ray-traced OctoMap that removes surfaces no longer there '
                                          '(e.g. dug terrain); settings in config/dynamic_map.yaml.'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in an existing map instead of mapping.'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_vislam.db'),
                              description='RTAB-Map database (map) file.'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap, e.g. "--Vis/MaxFeatures 800".'),

        camera,
        imu_filter,
        odometry,
        slam_mapping,
        slam_localization,
        rviz,
        viewer,
        rtabmap_gui,
    ])
