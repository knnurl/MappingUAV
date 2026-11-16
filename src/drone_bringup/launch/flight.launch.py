# filepath: src/drone_bringup/launch/flight.launch.py
"""The one flight bringup. Layers are switched on by arguments, lowest first;
each layer needs the gate token named beside it (masterplan §1.1). Launching a
layer on the vehicle before its token is a violation.

  always     perception: XRCE agent, Mid-360 + FAST-LIO, px4_odom_bridge
             geofence watchdog (flight settings: silent streams trip it)
  as2:=true       Aerostack2 platform, estimator, controller, behaviours   [gate4]
  mapping:=true   map_interface, map_saver, lio_health_guard, planner      [gate5]
  explorer:=true  frontier_explorer (needs as2 and mapping)                [gate6]

  ros2 launch drone_bringup flight.launch.py \
      serial_dev:=/dev/serial/by-id/usb-FTDI_TTL232RG-VSW3V3_FTADW2E6-if00-port0

All boxes come from venue_params (written by scripts/venue_boxes.py from
config/venue.yaml). Then run scripts/preflight_check.py before arming.
Restart this launch between flights and after any Offboard exit: the platform
forwards no Aerostack2 command again until it is restarted (offboard guard).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    cfg = os.path.join(share, 'config')
    launch_dir = os.path.join(share, 'launch')
    venue = LaunchConfiguration('venue_params')
    as2 = LaunchConfiguration('as2')
    mapping = LaunchConfiguration('mapping')
    explorer = LaunchConfiguration('explorer')

    def include(name, **args):
        return PythonLaunchDescriptionSource(os.path.join(launch_dir, name)), args

    src, args = include('gate3_fusion.launch.py')
    perception = IncludeLaunchDescription(
        src, launch_arguments={'serial_dev': LaunchConfiguration('serial_dev')}.items())
    watchdog = Node(
        package='geofence_watchdog', executable='watchdog_node', name='geofence_watchdog',
        output='screen',
        parameters=[os.path.join(cfg, 'geofence.yaml'), venue,
                    {'require_streams_after_s': 10.0}])
    src, _ = include('gate5_as2.launch.py')
    as2_stack = IncludeLaunchDescription(src, condition=IfCondition(as2))
    src, _ = include('exploration.launch.py')
    autonomy = IncludeLaunchDescription(
        src, condition=IfCondition(mapping),
        launch_arguments={'venue_params': venue, 'explorer': explorer,
                          'save_map': LaunchConfiguration('save_map')}.items())
    bad_combo = PythonExpression([
        "'", explorer, "' == 'true' and not ('", as2, "' == 'true' and '", mapping,
        "' == 'true')"])
    return LaunchDescription([
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument(
            'venue_params', default_value=os.path.join(cfg, 'venue_params.yaml')),
        DeclareLaunchArgument('as2', default_value='false'),
        DeclareLaunchArgument('mapping', default_value='false'),
        DeclareLaunchArgument('explorer', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='true'),
        LogInfo(condition=IfCondition(bad_combo),
                msg='WARNING: explorer:=true needs as2:=true and mapping:=true; '
                    'the explorer will refuse to start a mission.'),
        perception, watchdog, as2_stack, autonomy,
    ])
