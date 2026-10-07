# Fusion mapping with an Intel RealSense D455: the most accurate single-camera mode.
#
# The IR projector alternates on/off every frame (stock ROS 2 Humble packages +
# scripts/emitter_splitter.py):
#   realsense2_camera     -> IR pair + depth + IMU, projector interleaved
#   emitter_splitter      -> projector-off IR -> /split/infra*, projector-on depth -> /split/depth
#   imu_filter_madgwick   -> IMU orientation on /imu/data
#   rtabmap_odom          -> stereo_odometry: visual-inertial tracking on clean IR
#   rtabmap_util          -> point_cloud_xyz: dense depth -> /lidar/points
#   rtabmap_sync          -> stereo_sync: IR pair -> /rgbd_image (rtabmap can't sync raw stereo + cloud)
#   rtabmap_slam          -> visual guess + ICP refinement of every link and loop closure,
#                            map built from the dense clouds
#   rviz2 + vislam_viewer -> map/scan/trajectory window + camera/landmarks window
#
# All node settings live in config/d455_fusion.yaml.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    localization = LaunchConfiguration('localization')
    config = LaunchConfiguration('config')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    database = {'database_path': LaunchConfiguration('database_path')}
    sim = LaunchConfiguration('sim')
    # Real camera: images come through the projector splitter. Isaac Sim: straight from
    # /camera (rendered IR has no projector dots).
    src = PythonExpression(["'/camera' if '", sim, "'.lower() == 'true' else '/split'"])

    stereo = [
        ('imu', '/imu/data'),
        ('left/image_rect', [src, '/infra1/image_rect_raw']),
        ('left/camera_info', [src, '/infra1/camera_info']),
        ('right/image_rect', [src, '/infra2/image_rect_raw']),
        ('right/camera_info', [src, '/infra2/camera_info']),
    ]

    camera = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='', name='camera', output='screen', emulate_tty=True,
        condition=UnlessCondition(sim),
        parameters=[config],
    )

    splitter = ExecuteProcess(
        condition=UnlessCondition(sim),
        cmd=['python3', os.path.join(REPO, 'scripts', 'emitter_splitter.py')],
        output='screen',
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
        remappings=stereo,
    )

    depth_to_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[config],
        remappings=[
            ('depth/image', [src, '/depth/image_rect_raw']),
            ('depth/camera_info', [src, '/depth/camera_info']),
            ('cloud', '/lidar/points'),
        ],
    )

    stereo_sync = Node(
        package='rtabmap_sync', executable='stereo_sync', output='screen',
        parameters=[config],
        remappings=stereo + [('rgbd_image', '/rgbd_image')],
    )

    slam_inputs = [('imu', '/imu/data'), ('rgbd_image', '/rgbd_image'),
                   ('scan_cloud', '/lidar/points')]

    slam_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(localization),
        parameters=[config, database],
        arguments=[PythonExpression(["'--delete_db_on_start' if '",
                                     LaunchConfiguration('new_map'),
                                     "'.lower() == 'true' else ''"]),
                   rtabmap_args],
        remappings=slam_inputs,
    )

    slam_localization = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=IfCondition(localization),
        parameters=[config, database, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        arguments=[rtabmap_args],
        remappings=slam_inputs,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='fusion_map', output='log',
        condition=IfCondition(viz),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    viewer = ExecuteProcess(
        condition=IfCondition(viz),
        cmd=['python3', os.path.join(REPO, 'scripts', 'vislam_viewer.py'),
             '--ros-args', '-r', ['/camera/infra1/image_rect_raw:=', src, '/infra1/image_rect_raw'],
             '-p', ['use_sim_time:=', sim]],
        additional_env={'GDK_BACKEND': 'x11'},
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config',
                              default_value=os.path.join(REPO, 'config', 'd455_fusion.yaml'),
                              description='Parameter file with all settings.'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open the map (RViz) and camera/landmarks windows.'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(REPO, 'rviz', 'lidar_map.rviz'),
                              description='RViz config for the map window.'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Wipe database_path and start a fresh map (false = extend).'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in an existing map instead of mapping.'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_fusion.db'),
                              description='RTAB-Map database (map) file.'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap.'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Isaac Sim: no camera driver/splitter, simulated clock.'),
        SetParameter(name='use_sim_time', value=sim),

        camera,
        splitter,
        imu_filter,
        odometry,
        depth_to_cloud,
        stereo_sync,
        slam_mapping,
        slam_localization,
        rviz,
        viewer,
    ])
