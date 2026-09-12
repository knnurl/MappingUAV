# filepath: src/drone_bringup/launch/gate5_as2.launch.py
"""Gate 5: Aerostack2 skeleton on top of the Gate 3 stack (WP-B).

INTEGRATION PRECONDITION: gate4_tethered_hover CONFIRMED in
docs/gate_status.yaml. Launching this on the vehicle before that token is a
critical violation (masterplan §1.1). Bench-launching without the FC powered
is safe: the platform idles without /fmu/out traffic.

Composition: pixhawk platform + AS2 state estimator (raw_odometry plugin
reading the PX4-fused estimate — ONE estimator of record) + motion
controller (PID speed controller plugin, AS2 default position pipeline) +
motion behaviors (takeoff / go_to / land / hover). Trajectory/ACRO paths
deliberately not enabled [FIXED].

SOLE EV WRITER [FIXED]: px4_odom_bridge is the only node allowed to send on
/fmu/in/vehicle_visual_odometry. This launch refuses to start unless
platform_pixhawk.yaml sets external_odom: false. Expected at Gate 5:
`ros2 topic info -v /fmu/in/vehicle_visual_odometry` lists TWO publishers,
px4_odom_bridge (RELIABLE, sending ~10 Hz) and <namespace>/platform
(BEST_EFFORT, silent: pixhawk_platform.cpp:134 creates it unconditionally).
Only px4_odom_bridge may be sending; the bridge logs every other publisher.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def assert_external_odom_off(platform_cfg_path):
    """Refuse to launch unless the platform's EV publisher stays silent.

    external_odom:=true makes the platform republish AS2's odometry, which is
    EKF2's own output, back into EKF2's EV input at 100 Hz (a feedback loop
    that outvotes LIO 10:1). A missing key also fails: the platform declares the
    parameter without a default.
    """
    with open(platform_cfg_path) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    if params.get('external_odom') is not False:
        raise RuntimeError(
            f'{platform_cfg_path}: external_odom must be false (got '
            f'{params.get("external_odom")!r}). px4_odom_bridge is the sole writer of '
            '/fmu/in/vehicle_visual_odometry [FIXED, masterplan §7].')


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    as2_cfg = os.path.join(bringup_share, 'config', 'as2')
    platform_share = get_package_share_directory('as2_platform_pixhawk')
    assert_external_odom_off(os.path.join(as2_cfg, 'platform_pixhawk.yaml'))

    namespace = LaunchConfiguration('namespace')

    # Direct Node instead of upstream pixhawk_launch.py: that launcher's
    # DeclareLaunchArgumentsFromConfigFile stringifies the empty fmu_prefix
    # ('') into literal quote characters -> InvalidTopicNameError. The node
    # itself is fine with our params file (verified standalone). Upstream
    # source stays untouched; only their launch wrapper is bypassed.
    platform = Node(
        package='as2_platform_pixhawk',
        executable='as2_platform_pixhawk_node',
        name='platform',
        namespace=namespace,
        output='screen',
        parameters=[
            {'control_modes_file': os.path.join(
                platform_share, 'config', 'control_modes.yaml')},
            os.path.join(as2_cfg, 'platform_pixhawk.yaml'),
        ],
    )

    state_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_state_estimator'),
            'launch', 'state_estimator_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'plugin_name': 'raw_odometry',
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_motion_controller'),
            'launch', 'controller_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'plugin_name': 'pid_speed_controller',
            'plugin_config_file': os.path.join(as2_cfg, 'pid_speed_controller.yaml'),
        }.items(),
    )

    # Plugin choice [FIXED]: position/speed plugins only; every *_trajectory
    # plugin stays unused (masterplan §7: no AS2 trajectory paths yet).
    behaviors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_behaviors_motion'),
            'launch', 'motion_behaviors_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'takeoff_plugin_name': 'takeoff_plugin_position',
            'go_to_plugin_name': 'go_to_plugin_position',
            'land_plugin_name': 'land_plugin_speed',
            'follow_path_plugin_name': 'follow_path_plugin_position',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        platform,
        state_estimator,
        controller,
        behaviors,
    ])
