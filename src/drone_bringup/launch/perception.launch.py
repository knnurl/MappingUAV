# filepath: src/drone_bringup/launch/perception.launch.py
"""Perception pipeline for the systemd service (WP-G): gate2 (Livox +
FAST-LIO2) + the odometry frame bridge + static TFs. Identical to
gate3_fusion.launch.py MINUS the XRCE agent, which runs as its own systemd
unit (drone-xrce-agent.service) so a perception crash never drops the FC link.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'gate2_lio.launch.py')))

    odom_bridge = Node(
        package='px4_odom_bridge',
        executable='odom_bridge_node',
        name='px4_odom_bridge',
        output='screen',
        parameters=[{
            'lio_odom_topic': '/Odometry',
        }],
    )

    # Same placeholder-and-identity TFs as gate3_fusion; see the
    # architectural note there. [MEASURE] still pending on the lidar mount.
    tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_livox',
        arguments=['--x', '9.99', '--y', '9.99', '--z', '9.99',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'livox_frame'],
    )
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
    )

    return LaunchDescription([lio, odom_bridge, tf_base_to_livox, tf_map_to_odom])
