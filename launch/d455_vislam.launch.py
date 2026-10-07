# Stereo visual-inertial SLAM for an Intel RealSense D455.
#
# Pipeline (all stock ROS 2 Humble packages, no source builds):
#   realsense2_camera     -> left/right IR images (emitter off) + united gyro/accel IMU
#   imu_filter_madgwick   -> fuses gyro+accel into an orientation quaternion (/imu/data)
#   rtabmap_odom          -> stereo odometry, IMU used for motion prior + gravity alignment
#   rtabmap_slam          -> graph SLAM with loop closure + IMU gravity constraints
#   rtabmap_viz           -> optional GUI
#
# Usage:
#   ros2 launch launch/d455_vislam.launch.py
#   ros2 launch launch/d455_vislam.launch.py viz:=false                 # headless (robot)
#   ros2 launch launch/d455_vislam.launch.py localization:=true         # reuse saved map
#   ros2 launch launch/d455_vislam.launch.py database_path:=~/arena.db  # custom map file
#   ros2 launch launch/d455_vislam.launch.py rtabmap_args:=--delete_db_on_start

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rtabmap_args = LaunchConfiguration('rtabmap_args')

    parameters = {
        'frame_id': LaunchConfiguration('frame_id'),
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'approx_sync': False,       # IR pair is hardware-synced
        'wait_imu_to_init': True,   # start odometry gravity-aligned
        'database_path': database_path,
    }

    remappings = [
        ('imu', '/imu/data'),
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': '',
            'camera_name': 'camera',
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'depth_module.infra_profile': '848,480,30',
            'enable_sync': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200',
            'accel_fps': '200',
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
        }.items(),
    )

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
        remappings=[('imu/data_raw', '/camera/imu')],
    )

    odometry = Node(
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=[parameters],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(localization),
        parameters=[parameters],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam_localization = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=IfCondition(localization),
        parameters=[parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    visualizer = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        condition=IfCondition(viz),
        parameters=[parameters, {'odometry_node_name': 'stereo_odometry'}],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='camera_link',
                              description='Robot frame tracked by odometry (e.g. base_link).'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Launch the rtabmap_viz GUI.'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in an existing map instead of mapping.'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_vislam.db'),
                              description='RTAB-Map database (map) file.'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap, e.g. --delete_db_on_start.'),
        DeclareLaunchArgument('unite_imu_method', default_value='2',
                              description='0-None, 1-copy, 2-linear_interpolation.'),

        # Projector dots break stereo feature tracking; turn the IR emitter off.
        SetParameter(name='depth_module.emitter_enabled', value=0),

        camera,
        imu_filter,
        odometry,
        slam_mapping,
        slam_localization,
        visualizer,
    ])
