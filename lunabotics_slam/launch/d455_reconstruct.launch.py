# Colour 3D reconstruction with an Intel RealSense D455 (stock ROS 2 Humble packages).
#
#   realsense2_camera     -> colour + depth aligned to it + IMU, projector always on
#   imu_filter_madgwick   -> IMU orientation on /imu/data
#   rtabmap_odom          -> rgbd_odometry: visual-inertial tracking on colour + depth
#   rtabmap_slam          -> pose graph with loop closure; every keyframe keeps its colour
#                            image and depth, which is what the mesh export textures from
#   rviz2 + vislam_viewer -> coloured map window + camera/features window
#
# All node settings live in config/d455_reconstruct.yaml. The mesh itself is exported
# from the saved database after the run (run/physical/reconstruct.sh does both).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    config = LaunchConfiguration('config')
    rtabmap_args = LaunchConfiguration('rtabmap_args')

    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
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
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[config],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[config, {'database_path': LaunchConfiguration('database_path')}],
        arguments=[PythonExpression(["'--delete_db_on_start' if '",
                                     LaunchConfiguration('new_map'),
                                     "'.lower() == 'true' else ''"]),
                   rtabmap_args],
        remappings=remappings,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='reconstruct_map', output='log',
        condition=IfCondition(viz),
        arguments=['-d', os.path.join(REPO, 'rviz', 'reconstruct.rviz')],
        remappings=[('/vislam/map_cloud', '/cloud_map')],
    )

    viewer = ExecuteProcess(
        condition=IfCondition(viz),
        cmd=['python3', os.path.join(REPO, 'scripts', 'vislam_viewer.py'),
             '--ros-args', '-r', '/camera/infra1/image_rect_raw:=/camera/color/image_raw'],
        additional_env={'GDK_BACKEND': 'x11'},
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config',
                              default_value=os.path.join(REPO, 'config', 'd455_reconstruct.yaml'),
                              description='Parameter file with all settings.'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open the map (RViz) and camera/features windows.'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Wipe database_path and start a fresh scan (false = extend).'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_reconstruct.db'),
                              description='RTAB-Map database the scan is saved to (and exported from).'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap.'),

        camera,
        imu_filter,
        odometry,
        slam,
        rviz,
        viewer,
    ])
