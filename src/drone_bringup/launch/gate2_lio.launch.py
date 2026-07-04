# filepath: src/drone_bringup/launch/gate2_lio.launch.py
"""Gate 2: Mid-360 driver in CustomMsg mode + FAST-LIO2, headless.

Does NOT start the PX4 bridge; Gate 2 is a pure perception bench test.
Verified against cloned sources 2026-07-04:
- livox_ros_driver2_node parameter names match (livox_ros_driver2.cpp).
- FAST_LIO ROS2 mapping.launch.py takes config_path (a DIRECTORY) plus
  config_file (a FILENAME resolved inside config_path) — not a full path.
  The handover spec's draft passed a full path as config_file; corrected here.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    fast_lio_share = get_package_share_directory('fast_lio')

    livox_config = os.path.join(bringup_share, 'config', 'MID360_config.json')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,        # 1 = livox CustomMsg. REQUIRED by FAST-LIO. [FIXED]
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': livox_config,
        }],
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_path': os.path.join(bringup_share, 'config'),
            'config_file': 'fast_lio_mid360_x500.yaml',
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription([livox_driver, fast_lio])
