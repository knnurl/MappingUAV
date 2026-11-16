# filepath: src/drone_bringup/launch/exploration.launch.py
"""Mapping, planning and exploration layer (WP-C, WP-D, WP-F).

INTEGRATION PRECONDITION: gate6_planner_in_loop CONFIRMED for the explorer,
gate5_as2_behaviors CONFIRMED for the map server and the planner. Launching on
the vehicle before those tokens is a violation (masterplan §1.1).

Starts: map_interface (occupancy map), map_saver (3D map checkpoints on disk),
lio_health_guard, go_to_with_avoidance (planner), and frontier_explorer when
explorer:=true. Does NOT start perception, the XRCE agent, the geofence
watchdog or Aerostack2: those have their own launch files and must already be
running (the explorer refuses to start a mission otherwise).

venue_params: a parameter file written by scripts/venue_boxes.py. It is loaded
last, so the map bounds, planning box and explorer box all come from one place.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('drone_bringup'), 'config')
    venue = LaunchConfiguration('venue_params')
    map_cfg = os.path.join(cfg, 'map_interface.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'venue_params', default_value=os.path.join(cfg, 'venue_params.yaml')),
        DeclareLaunchArgument('explorer', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='true'),
        # ~/.local carries a numpy 2 that breaks the system scipy the explorer
        # and planner use (found 2026-11-16).
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        Node(package='map_interface', executable='map_server_node', name='map_interface',
             output='screen', parameters=[map_cfg, venue]),
        Node(package='map_interface', executable='map_saver_node', name='map_saver',
             output='screen', parameters=[map_cfg],
             condition=IfCondition(LaunchConfiguration('save_map'))),
        Node(package='lio_health_guard', executable='guard_node', name='lio_health_guard',
             output='screen'),
        Node(package='planner_shim', executable='go_to_with_avoidance',
             name='go_to_with_avoidance', output='screen',
             parameters=[os.path.join(cfg, 'planning', 'go_to_with_avoidance.yaml'), venue]),
        Node(package='frontier_explorer', executable='explorer_node',
             name='frontier_explorer', output='screen',
             parameters=[os.path.join(cfg, 'exploration', 'frontier_explorer.yaml'), venue],
             condition=IfCondition(LaunchConfiguration('explorer'))),
    ])
