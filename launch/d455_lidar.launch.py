# "Simulated 3D LiDAR" mapping with an Intel RealSense D455.
#
# The depth stream is turned into a point cloud and handled exactly like a 3D LiDAR,
# following rtabmap_examples/lidar3d.launch.py (stock ROS 2 Humble packages):
#   realsense2_camera     -> depth (IR projector on) + united gyro/accel IMU
#   rtabmap_util          -> point_cloud_xyz: depth -> /lidar/points; imu_to_tf: ICP guess frame
#   imu_filter_madgwick   -> IMU orientation on /imu/data
#   rtabmap_odom          -> icp_odometry: scan-to-map ICP
#   rtabmap_slam          -> scan-based graph SLAM, map built from the scans
#   rviz2                 -> map, live scan, trajectory
#
# All node settings live in config/d455_lidar.yaml.
#
# Usage:
#   ros2 launch launch/d455_lidar.launch.py
#   ros2 launch launch/d455_lidar.launch.py viz:=false
#   ros2 launch launch/d455_lidar.launch.py localization:=true
#   ros2 launch launch/d455_lidar.launch.py new_map:=false

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
    database = {'database_path': LaunchConfiguration('database_path')}

    remappings = [
        ('imu', '/imu/data'),
        ('scan_cloud', '/lidar/points'),
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

    depth_to_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[config],
        remappings=[
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', '/lidar/points'),
        ],
    )

    imu_to_tf = Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[config],
        remappings=[('imu/data', '/imu/data')],
    )

    odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[config],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(localization),
        parameters=[config, database],
        arguments=[PythonExpression(["'--delete_db_on_start' if '",
                                     LaunchConfiguration('new_map'),
                                     "'.lower() == 'true' else ''"]),
                   rtabmap_args],
        remappings=remappings,
    )

    slam_localization = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=IfCondition(localization),
        parameters=[config, database, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='lidar_map', output='log',
        condition=IfCondition(viz),
        arguments=['-d', os.path.join(REPO, 'rviz', 'lidar_map.rviz')],
    )

    # Headless: only publishes the smooth trajectory and camera frustum for RViz.
    trajectory = ExecuteProcess(
        condition=IfCondition(viz),
        cmd=['python3', os.path.join(REPO, 'scripts', 'vislam_viewer.py'), '--no-window'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config',
                              default_value=os.path.join(REPO, 'config', 'd455_lidar.yaml'),
                              description='Parameter file with all camera, cloud, ICP and SLAM settings.'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open the RViz map window.'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Wipe database_path and start a fresh map (false = extend).'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in an existing map instead of mapping.'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_lidar.db'),
                              description='RTAB-Map database (map) file.'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap, e.g. "--Icp/VoxelSize 0.03".'),

        camera,
        imu_filter,
        depth_to_cloud,
        imu_to_tf,
        odometry,
        slam_mapping,
        slam_localization,
        rviz,
        trajectory,
    ])
