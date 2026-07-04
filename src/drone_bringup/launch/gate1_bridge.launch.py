# filepath: src/drone_bringup/launch/gate1_bridge.launch.py
"""Gate 1: bring up the Micro XRCE-DDS agent on the Pixhawk 6C TELEM2 serial link.

Wiring assumption [MEASURE/VERIFY]: Pixhawk TELEM2 <-> Jetson 40-pin UART1
(/dev/ttyTHS1, pins 8 TXD / 10 RXD, 3.3V, TX->RX crossed, common GND).
If a USB-serial adapter is used instead, override serial_dev at launch.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_dev = LaunchConfiguration('serial_dev')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud', default_value='921600'),

        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial',
                 '--dev', serial_dev,
                 '-b', baud],
            name='micro_xrce_agent',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
