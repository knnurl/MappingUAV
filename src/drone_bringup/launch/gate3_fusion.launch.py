# filepath: src/drone_bringup/launch/gate3_fusion.launch.py
"""Gate 3: Gate 1 + Gate 2 + odometry frame bridge + static TFs.

IMPORTANT ARCHITECTURAL NOTE, do not refactor this away:
The static_transform_publishers below serve visualization and future sensor
extrinsics ONLY. They are NOT part of the estimation data path. The actual
ENU/FLU -> NED/FRD conversion for PX4 happens numerically inside
px4_odom_bridge, because PX4 consumes px4_msgs/VehicleOdometry field values,
not TF lookups. A static TF cannot perform this job.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'gate1_bridge.launch.py')))

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'gate2_lio.launch.py')))

    odom_bridge = Node(
        package='px4_odom_bridge',
        executable='odom_bridge_node',
        name='px4_odom_bridge',
        output='screen',
        parameters=[{
            'lio_odom_topic': '/Odometry',   # verified: laserMapping.cpp publishes "/Odometry"
        }],
    )

    # base_link (FLU, flight controller position) -> livox_frame.
    # [MEASURED 2026-09-14, operator]: the Mid-360 centre sits ~85 mm above the
    # Pixhawk 6C, horizontally inline, so (0, 0, +0.085) m in FLU.
    # Visualisation only: the estimation path does not use this TF (see the note
    # at the top). Before it is used for EKF2, refine it for the sensor's own
    # internal offset: FAST-LIO's body frame is the Mid-360's built-in IMU, not
    # the LiDAR centre (fast_lio_mid360_x500.yaml extrinsic_T), and EKF2's
    # EKF2_EV_POS_X/Y/Z (FRD, so z = -0.085) are still 0 on the FC.
    tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_livox',
        arguments=['--x', '0', '--y', '0', '--z', '0.085',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'livox_frame'],
    )

    # odom (FAST-LIO world, ENU) is the root visualization frame. Identity map->odom
    # placeholder for later global-mapping work; harmless now.
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
    )

    return LaunchDescription([bridge, lio, odom_bridge, tf_base_to_livox, tf_map_to_odom])
