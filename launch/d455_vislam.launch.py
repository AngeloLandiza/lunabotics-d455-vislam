# Stereo visual-inertial SLAM for an Intel RealSense D455.
#
# Pipeline (all stock ROS 2 Humble packages, no source builds):
#   realsense2_camera     -> left/right IR images (emitter off) + united gyro/accel IMU
#   imu_filter_madgwick   -> fuses gyro+accel into an orientation quaternion (/imu/data)
#   rtabmap_odom          -> stereo odometry, IMU used for motion prior + gravity alignment
#   rtabmap_slam          -> graph SLAM with loop closure + IMU gravity constraints
#   rviz2 + vislam_viewer -> map/trajectory window + camera/landmarks window
#
# Usage:
#   ros2 launch launch/d455_vislam.launch.py
#   ros2 launch launch/d455_vislam.launch.py viz:=false                 # headless (robot)
#   ros2 launch launch/d455_vislam.launch.py rtabmap_viz:=true          # stock RTAB-Map GUI too
#   ros2 launch launch/d455_vislam.launch.py localization:=true         # reuse saved map
#   ros2 launch launch/d455_vislam.launch.py database_path:=~/arena.db  # custom map file
#   ros2 launch launch/d455_vislam.launch.py new_map:=false              # extend saved map

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # RTAB-Map core parameters (strings, as RTAB-Map expects).
    odom_parameters = {
        # Re-initialize after 1 failed frame instead of staying lost forever;
        # rtabmap starts a new session and re-links it on the next loop closure.
        'Odom/ResetCountdown': '1',
        # Fewer features: ~39 ms/frame instead of ~52 ms, still ~150 inliers.
        'Vis/MaxFeatures': '500',
        'OdomF2M/MaxSize': '1500',
    }
    slam_parameters = {
        # Cleaner dense map from the stereo pair: SGBM disparity, short range
        # (stereo depth error grows with distance^2), 3 cm voxels, outlier removal.
        'Stereo/DenseStrategy': '1',
        'Grid/RangeMax': '4.0',
        'Grid/CellSize': '0.03',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '8',
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
            # Short fixed exposure: auto-exposure picks 15-20 ms indoors with the
            # emitter off, which motion-blurs features and loses tracking.
            # (The firmware ignores depth_module.auto_exposure_limit on this unit.)
            'depth_module.enable_auto_exposure': LaunchConfiguration('auto_exposure'),
            'depth_module.exposure': LaunchConfiguration('ir_exposure'),
            'depth_module.gain': LaunchConfiguration('ir_gain'),
        }.items(),
    )

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
        remappings=[('imu/data_raw', '/camera/imu')],
    )

    odometry = Node(
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=[parameters, odom_parameters],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    slam_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(localization),
        parameters=[parameters, slam_parameters],
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
        parameters=[parameters, slam_parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        arguments=[rtabmap_args],
        remappings=remappings,
    )

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    rviz = Node(
        package='rviz2', executable='rviz2', name='vislam_map', output='log',
        condition=IfCondition(viz),
        arguments=['-d', os.path.join(repo, 'rviz', 'vislam_map.rviz')],
    )

    viewer = ExecuteProcess(
        condition=IfCondition(viz),
        cmd=['python3', os.path.join(repo, 'scripts', 'vislam_viewer.py')],
        additional_env={'GDK_BACKEND': 'x11'},  # same XWayland path as RViz
        output='screen',
    )

    rtabmap_gui = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        parameters=[parameters, {'odometry_node_name': 'stereo_odometry'}],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='camera_link',
                              description='Robot frame tracked by odometry (e.g. base_link).'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open the map (RViz) and camera/landmarks windows.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Also open the stock rtabmap_viz GUI.'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Mapping mode: wipe database_path and start a fresh map '
                                          '(false = extend the existing map).'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize in an existing map instead of mapping.'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/d455_vislam.db'),
                              description='RTAB-Map database (map) file.'),
        DeclareLaunchArgument('rtabmap_args', default_value='',
                              description='Extra args for odometry and rtabmap, e.g. --delete_db_on_start.'),
        DeclareLaunchArgument('auto_exposure', default_value='false',
                              description='IR auto-exposure (blurs under motion indoors).'),
        DeclareLaunchArgument('ir_exposure', default_value='5000',
                              description='IR exposure in microseconds when auto_exposure is false.'),
        DeclareLaunchArgument('ir_gain', default_value='64',
                              description='IR gain (16-248) when auto_exposure is false; raise in dim light.'),
        DeclareLaunchArgument('unite_imu_method', default_value='2',
                              description='0-None, 1-copy, 2-linear_interpolation.'),

        # Projector dots break stereo feature tracking; turn the IR emitter off.
        SetParameter(name='depth_module.emitter_enabled', value=0),

        camera,
        imu_filter,
        odometry,
        slam_mapping,
        slam_localization,
        rviz,
        viewer,
        rtabmap_gui,
    ])
