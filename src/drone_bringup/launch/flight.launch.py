# filepath: src/drone_bringup/launch/flight.launch.py
"""The one flight bringup. Layers are switched on by arguments, lowest first;
each layer needs the gate token named beside it (masterplan §1.1). Launching a
layer on the vehicle before its token is a violation.

  always     perception: XRCE agent, Mid-360 + FAST-LIO, px4_odom_bridge
             geofence watchdog (flight settings: silent streams trip it)
  as2:=true       Aerostack2 platform, estimator, controller, behaviours   [gate4]
  mapping:=true   map_interface, map_saver, lio_health_guard, planner      [gate5]
  explorer:=true  frontier_explorer (needs as2 and mapping)                [gate6]
  lio:=fork       the FASTLIO2_ROS2 fork instead of upstream FAST-LIO; only
                  after docs/lio_swap_runbook.txt (default: upstream)

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


def generate_launch_description():
    share = get_package_share_directory('drone_bringup')
    cfg = os.path.join(share, 'config')
    launch_dir = os.path.join(share, 'launch')
    venue = LaunchConfiguration('venue_params')
    as2 = LaunchConfiguration('as2')
    mapping = LaunchConfiguration('mapping')
    explorer = LaunchConfiguration('explorer')
    lio = LaunchConfiguration('lio')

    def include(name, **args):
        return PythonLaunchDescriptionSource(os.path.join(launch_dir, name)), args

    src, args = include('gate3_fusion.launch.py')
    perception = IncludeLaunchDescription(
        src, launch_arguments={'serial_dev': LaunchConfiguration('serial_dev'), 'lio': lio}.items())
    src, _ = include('gate5_as2.launch.py')
    as2_stack = IncludeLaunchDescription(src, condition=IfCondition(as2))
    # Watchdog always; map, planner and explorer per argument (autonomy.launch.py).
    src, _ = include('autonomy.launch.py')
    autonomy = IncludeLaunchDescription(
        src, launch_arguments={'venue_params': venue, 'lio': lio, 'mapping': mapping,
                               'explorer': explorer,
                               'save_map': LaunchConfiguration('save_map')}.items())
    bad_combo = PythonExpression([
        "'", explorer, "' == 'true' and not ('", as2, "' == 'true' and '", mapping,
        "' == 'true')"])
    return LaunchDescription([
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument(
            'venue_params', default_value=os.path.join(cfg, 'venue_params.yaml')),
        DeclareLaunchArgument('as2', default_value='false'),
        DeclareLaunchArgument('lio', default_value='upstream',
                              description='upstream (hku-mars FAST_LIO) or fork (FASTLIO2_ROS2)'),
        DeclareLaunchArgument('mapping', default_value='false'),
        DeclareLaunchArgument('explorer', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='true'),
        LogInfo(condition=IfCondition(bad_combo),
                msg='WARNING: explorer:=true needs as2:=true and mapping:=true; '
                    'the explorer will refuse to start a mission.'),
        perception, autonomy, as2_stack,
    ])
