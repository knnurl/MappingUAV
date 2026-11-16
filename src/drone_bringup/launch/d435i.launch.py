#!/usr/bin/env python3
"""D435i bring-up for the drone: color + depth IMAGE views. Pointcloud and camera IMU off.

Role [operator decision 2026-09-12]: camera and depth views only. The D435i is
not a mapping/planning input; 3D data comes from the Mid-360. Nothing on the
vehicle subscribes to these topics.

Usage:
    source ~/colcon_ws/install/setup.bash
    ros2 launch drone_bringup d435i.launch.py

Views (subscribe off-board, same ROS_DOMAIN_ID):
    /camera/camera/color/image_raw          640x480@30
    /camera/camera/depth/image_rect_raw     ~424x240@30 (16UC1, mm; decimation x2)
  Raw 640x480 RGB at 30 Hz is ~220 Mbit/s: over WiFi use image_transport
  compressed / compressedDepth (image_transport_plugins) or lower the fps, or
  the camera will starve telemetry on the shared DDS link. Never bag raw images
  in flight.

Jetson notes (L4T R36.5 / JetPack 6, kernel 5.15-tegra):
  * This librealsense uses the V4L2 backend and the ARM-NEON pointcloud filter,
    so the enable param is 'pointcloud__neon_.enable' -- NOT 'pointcloud.enable'
    (the plain name is silently ignored on aarch64).
  * The camera IMU does not work on the stock Jetson kernel (missing hid-sensor
    modules) and is intentionally disabled. IMU comes from the Pixhawk 6C.

Verified 2026-09-12: depth 848x480@30, color 1280x720@30,
pointcloud /camera/camera/depth/color/points @ 30 Hz (~355k pts, XYZRGB).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pointcloud', default_value='false',
            description='Colored pointcloud stream (true/false). Off: views only; '
                        'measured 30-59% of one core when on.'),
        DeclareLaunchArgument(
            'decimation', default_value='true',
            description='Depth decimation filter [operator: on, magnitude 2]. Lowers the '
                        'depth IMAGE resolution for all consumers (~424x240 at x2) and '
                        'cuts CPU. false => native 848x480.'),
        DeclareLaunchArgument(
            'decimation_magnitude', default_value='2',
            description='Decimation factor, valid 2-8 (2 => depth ~424x240, ~32k pts).'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                # --- colored pointcloud (ARM NEON filter name!) ---
                'pointcloud__neon_.enable': ParameterValue(LaunchConfiguration('pointcloud'), value_type=bool),
                # texture source: 2 = color stream. Without this it defaults to
                # "Any" and the cloud comes out uncolored (all-black RGB).
                'pointcloud__neon_.stream_filter': 2,
                # 'pointcloud__neon_.ordered_pc': False,           # True => organized HxW cloud
                # 'pointcloud__neon_.allow_no_texture_points': True,

                # --- streams ---
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,

                # --- camera IMU OFF (Pixhawk 6C supplies IMU; HID unsupported here) ---
                'enable_gyro': False,
                'enable_accel': False,

                # --- performance: decimation filter (biggest CPU lever) ---
                # measured: default ~59% of 1 core (173k pts) -> decim x2 ~30% (32k pts)
                'decimation_filter.enable': ParameterValue(
                    LaunchConfiguration('decimation'), value_type=bool),
                'decimation_filter.filter_magnitude': ParameterValue(
                    LaunchConfiguration('decimation_magnitude'), value_type=int),

                # --- resolution / fps ---
                # depth sensor runs at native 848x480x30; decimation x2 (default)
                # publishes ~424x240.
                # color pinned to 640x480x30: ~3x less USB bandwidth than 720p, negligible
                # CPU change but avoids frame drops on the shared USB3 bus.
                # 'depth_module.depth_profile': '848,480,30',
                'rgb_camera.color_profile': '640,480,30',

                # --- optional: aligned depth-in-color-frame image ---
                # 'align_depth.enable': True,

                # --- robustness: hardware-reset camera on start (adds ~3s) ---
                # 'initial_reset': True,
            }],
        ),
    ])
