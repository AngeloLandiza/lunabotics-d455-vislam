# Autonomous run: the step 2 research pieces as ROS 2 nodes (scripts/autonomy_core.py holds the shared logic and
# calls the sim-tested code in sim/ directly, so run from the repo checkout).
#
#   start_tags.py      rear camera -> start-zone pose from the two AprilTags        -> /autonomy/tag_fix
#   arena_localizer.py tag fix + wheel + gyro + four-wall matching on the L1          -> /autonomy/pose, TF
#   hazard_detector.py L1 + D455 -> log-odds boulders / craters (BP-1 sandbox)      -> /autonomy/hazards
#   mission_runner.py  dig -> dump cycles, tangent-graph routes, replanning           -> /cmd_vel, /arena/dig
#   rviz2              autonomy.rviz (arena frame, top view): hazards, route, estimated vs true pose, rear camera
#
# In the sim (run/sim/arena/arena.sh --autonomy): sim:=true. To test one piece alone, feed the others the truth:
#   pose_topic:=/ground_truth/base_pose   detector and planner use the sim's true pose instead of the localizer

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # lunabotics_slam (source checkout)


def node(script, *params):
    cmd = ['python3', os.path.join(PKG, 'scripts', script), '--ros-args',
           '-p', ['use_sim_time:=', LaunchConfiguration('sim')]]
    for p in params:
        cmd += ['-p', p]
    return ExecuteProcess(cmd=cmd, output='screen')


def generate_launch_description():
    pose = ['pose_topic:=', LaunchConfiguration('pose_topic')]
    mission = node('mission_runner.py', pose, ['seed:=', LaunchConfiguration('seed')],
                   ['cycles:=', LaunchConfiguration('cycles')], ['autostart:=', LaunchConfiguration('autostart')],
                   ['sim_dig:=', LaunchConfiguration('sim_dig')])
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('pose_topic', default_value='/autonomy/pose'),
        DeclareLaunchArgument('seed', default_value='0', description='mission goal draw'),
        DeclareLaunchArgument('cycles', default_value='3'),
        DeclareLaunchArgument('autostart', default_value='true', description='drive the mission (false: map only)'),
        DeclareLaunchArgument('sim_dig', default_value=LaunchConfiguration('sim'),
                              description='ask the sim to dig / dump on arrival (/arena/dig)'),
        DeclareLaunchArgument('wheel_scale', default_value='0.95',
                              description='encoder distance scale from the BP-1 slip calibration'),
        DeclareLaunchArgument('use_depth', default_value='true', description='D455 depth in the detector too'),
        DeclareLaunchArgument('rviz', default_value='true'),
        node('start_tags.py'),
        node('arena_localizer.py', ['wheel_scale:=', LaunchConfiguration('wheel_scale')]),
        node('hazard_detector.py', pose, ['use_depth:=', LaunchConfiguration('use_depth')]),
        mission,
        # the mission ends (done, or stuck) -> the whole run ends
        RegisterEventHandler(OnProcessExit(target_action=mission,
                                           on_exit=[EmitEvent(event=Shutdown(reason='mission ended'))])),
        ExecuteProcess(cmd=['rviz2', '-d', os.path.join(PKG, 'rviz', 'autonomy.rviz'), '--ros-args',
                            '-p', ['use_sim_time:=', LaunchConfiguration('sim')]],
                       output='screen', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
