# filepath: src/drone_bringup/launch/autonomy.launch.py
"""Everything above perception EXCEPT Aerostack2: the geofence watchdog (flight
settings) and, with mapping:=true, map_interface, map_saver, lio_health_guard,
the planner and (explorer:=true) the explorer.

Aerostack2 is deliberately separate (gate5_as2.launch.py / drone-as2.service):
it must be restarted after every Offboard exit, and restarting it must not
restart the watchdog or the map. flight.launch.py puts all of it together for a
single terminal.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    cfg = os.path.join(share, 'config')
    venue = LaunchConfiguration('venue_params')
    lio = LaunchConfiguration('lio')
    lio_params = PathJoinSubstitution([cfg, ['lio_', lio, '.yaml']])
    return LaunchDescription([
        DeclareLaunchArgument(
            'venue_params', default_value=os.path.join(cfg, 'venue_params.yaml')),
        DeclareLaunchArgument('lio', default_value='upstream'),
        DeclareLaunchArgument('mapping', default_value='false'),
        DeclareLaunchArgument('explorer', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='true'),
        Node(package='geofence_watchdog', executable='watchdog_node',
             name='geofence_watchdog', output='screen',
             parameters=[os.path.join(cfg, 'geofence.yaml'), venue, lio_params,
                         {'require_streams_after_s': 10.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'exploration.launch.py')),
            condition=IfCondition(LaunchConfiguration('mapping')),
            launch_arguments={'venue_params': venue, 'lio': lio,
                              'explorer': LaunchConfiguration('explorer'),
                              'save_map': LaunchConfiguration('save_map')}.items()),
    ])
