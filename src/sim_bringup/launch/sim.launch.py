# filepath: src/sim_bringup/launch/sim.launch.py
"""Closed-loop rig: the REAL autonomy stack against a kinematic fake PX4 and
a raycast LiDAR world. No vehicle, no XRCE agent, no FAST-LIO.

REFUSES to start unless ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID is not 0, so it
can never share a graph with the flight controller (PX4 uses domain 0).

Real: px4_odom_bridge, geofence_watchdog, Aerostack2 (patched platform, state
estimator, controller, behaviours), map_interface, lio_health_guard,
go_to_with_avoidance, frontier_explorer.
Fake: PX4 (fake_px4), Mid-360 + FAST-LIO (fake_lidar).

  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=73 ros2 launch sim_bringup sim.launch.py
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=73 ros2 run sim_bringup sim_mission
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or \
            os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError(
            'sim.launch.py runs only with ROS_LOCALHOST_ONLY=1 and a non-zero '
            'ROS_DOMAIN_ID, so the rig cannot reach the flight controller.')
    sim = get_package_share_directory('sim_bringup')
    bringup = get_package_share_directory('drone_bringup')
    venue = os.path.join(sim, 'config', 'venue_params_two_rooms.yaml')
    world = LaunchConfiguration('world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='two_rooms'),
        DeclareLaunchArgument('explorer', default_value='true'),
        DeclareLaunchArgument('save_map', default_value='false'),
        Node(package='sim_bringup', executable='fake_px4', output='screen'),
        Node(package='sim_bringup', executable='fake_lidar', output='screen',
             parameters=[{'world': world}]),
        Node(package='px4_odom_bridge', executable='odom_bridge_node',
             name='px4_odom_bridge', output='screen', parameters=[venue]),
        Node(package='geofence_watchdog', executable='watchdog_node',
             name='geofence_watchdog', output='screen',
             parameters=[os.path.join(bringup, 'config', 'geofence.yaml'), venue,
                         {'require_streams_after_s': 10.0}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'gate5_as2.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'exploration.launch.py')),
            launch_arguments={'venue_params': venue,
                              'explorer': LaunchConfiguration('explorer'),
                              'save_map': LaunchConfiguration('save_map')}.items()),
    ])
