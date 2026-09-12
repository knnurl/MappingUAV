# filepath: src/px4_odom_bridge/px4_odom_bridge/odom_bridge_node.py
"""Converts FAST-LIO2 nav_msgs/Odometry (ENU world, FLU body) into
px4_msgs/VehicleOdometry (NED world, FRD body) on /fmu/in/vehicle_visual_odometry.

Design decisions, do not alter without re-reading the handover spec:
- This node is the SOLE writer of /fmu/in/vehicle_visual_odometry; EKF2 is the
  one estimator of record. as2_platform_pixhawk also creates a publisher on
  this topic (unconditionally, pixhawk_platform.cpp:134) that must stay silent:
  external_odom:=false, enforced by gate5_as2.launch.py. Enabling it would feed
  EKF2's own output back into EKF2 at 100 Hz. This node logs every other
  publisher it discovers on the topic.
- Position + orientation only. Velocity and angular velocity are set NaN because
  EKF2_EV_CTRL=11 does not fuse them and the source twist frame is untrusted.
- Variances set NaN; EKF2_EV_NOISE_MD=1 makes PX4 use its fixed noise params.
- Frames: FAST-LIO's world x axis is the vehicle's STARTUP forward direction,
  not East. The ENU->NED swap therefore puts startup-forward on PX4 East and
  startup-left on PX4 North; PX4 heading reads ~ +pi/2 at startup. This is
  intended: AS2 converts NED->ENU with the same swap, so its odom frame equals
  FAST-LIO's world and /cloud_registered lines up with AS2 poses.
- timestamp_sample is the LIO measurement time (odom.header.stamp = scan end),
  not the callback time, so FAST-LIO's variable processing latency is not folded
  into EKF2's delay compensation; EKF2_EV_DELAY only covers the residual. This
  is valid because the Mid-360 runs unsynchronised: livox_ros_driver2 then
  stamps packets with Jetson system time on arrival, the same clock as now().
  TRAP: if PTP/gPTP is ever enabled on the Mid-360, the driver stamps with the
  LiDAR's PTP clock instead. That is only valid with phc2sys disciplining the
  Jetson system clock; otherwise the stamps will fail the age check below.
- Stamps in the future or older than max_sample_age_s are dropped, never sent.
- timestamp / timestamp_sample are ROS-clock microseconds; with UXRCE_DDS_SYNCT=1
  the PX4 uXRCE-DDS client shifts both into FC time.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

EV_TOPIC = '/fmu/in/vehicle_visual_odometry'

# Quaternions as (w, x, y, z).
# 180 deg rotation about (1,1,0)/sqrt(2): maps ENU world axes onto NED world axes.
Q_ENU_TO_NED = np.array([0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0])
# 180 deg rotation about body X: maps FLU body axes onto FRD body axes.
Q_FLU_TO_FRD = np.array([0.0, 1.0, 0.0, 0.0])


def q_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, (w,x,y,z) convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def enu_flu_to_ned_frd(position_enu, q_enu_flu):
    """Pose conversion. Returns (position_ned, q_ned_frd), q as (w,x,y,z)."""
    # Position: ENU -> NED is (x,y,z) -> (y, x, -z).
    x, y, z = position_enu
    # Orientation: q_ned_frd = Q_ENU_TO_NED * q_enu_flu * Q_FLU_TO_FRD
    q = q_mult(q_mult(Q_ENU_TO_NED, np.asarray(q_enu_flu, dtype=float)), Q_FLU_TO_FRD)
    q /= np.linalg.norm(q)
    return [float(y), float(x), float(-z)], [float(v) for v in q]


def stamp_to_us(stamp) -> int:
    """builtin_interfaces/Time -> integer microseconds."""
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


def sample_age_ok(sample_us: int, now_us: int, max_age_us: int) -> bool:
    """A LIO stamp is usable only if it is neither in the future nor stale."""
    return 0 <= now_us - sample_us <= max_age_us


def other_writers(publisher_infos, own_fqn: str) -> list:
    """Fully qualified names of every publisher on the topic except this node."""
    names = {f"{info.node_namespace.rstrip('/')}/{info.node_name}"
             for info in publisher_infos}
    names.discard(own_fqn)
    return sorted(names)


class OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')
        self.declare_parameter('lio_odom_topic', '/Odometry')
        self.declare_parameter('max_sample_age_s', 0.5)
        topic = self.get_parameter('lio_odom_topic').value
        self.max_age_us = int(self.get_parameter('max_sample_age_s').value * 1e6)

        # Subscribe with sensor-data-compatible QoS (FAST-LIO publishers vary).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Publish to /fmu/in with default RELIABLE QoS: compatible with the
        # agent-side subscriber. Do not change to BEST_EFFORT.
        self.pub = self.create_publisher(VehicleOdometry, EV_TOPIC, 10)
        self.sub = self.create_subscription(Odometry, topic, self.cb, sub_qos)

        self.msg_count = 0
        self.drop_count = 0
        self.known_writers = []
        self.create_timer(5.0, self.health)
        self.create_timer(2.0, self.check_writers)
        self.get_logger().info(f'Bridging {topic} -> {EV_TOPIC}')

    def cb(self, odom: Odometry):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        sample_us = stamp_to_us(odom.header.stamp)
        if not sample_age_ok(sample_us, now_us, self.max_age_us):
            self.drop_count += 1
            self.get_logger().warn(
                f'Dropping LIO odometry: stamp age {(now_us - sample_us) / 1e6:.3f} s '
                f'outside 0..{self.max_age_us / 1e6:.3f} s (clock mismatch? see docstring)',
                throttle_duration_sec=5.0)
            return

        out = VehicleOdometry()
        out.timestamp = now_us
        out.timestamp_sample = sample_us

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        out.position, out.q = enu_flu_to_ned_frd([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

        nan = float('nan')
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = [nan, nan, nan]
        out.angular_velocity = [nan, nan, nan]
        out.position_variance = [nan, nan, nan]
        out.orientation_variance = [nan, nan, nan]
        out.velocity_variance = [nan, nan, nan]
        out.quality = 0
        out.reset_counter = 0

        self.pub.publish(out)
        self.msg_count += 1

    def health(self):
        if self.msg_count == 0 and self.drop_count == 0:
            self.get_logger().warn('No LIO odometry received in last 5 s')
        elif self.drop_count:
            self.get_logger().warn(
                f'Last 5 s: {self.msg_count} forwarded, {self.drop_count} dropped (bad stamps)')
        self.msg_count = 0
        self.drop_count = 0

    def check_writers(self):
        writers = other_writers(
            self.get_publishers_info_by_topic(EV_TOPIC), self.get_fully_qualified_name())
        if writers == self.known_writers:
            return
        self.known_writers = writers
        if writers:
            self.get_logger().warn(
                f'Other publishers on {EV_TOPIC}: {", ".join(writers)}. This node must be '
                'the sole EV writer. From Gate 5 an as2_platform_pixhawk endpoint is '
                'expected and must stay silent (external_odom:=false).')
        else:
            self.get_logger().info(f'Sole publisher on {EV_TOPIC} again')


def main():
    rclpy.init()
    node = OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
