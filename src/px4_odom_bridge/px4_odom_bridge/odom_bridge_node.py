# filepath: src/px4_odom_bridge/px4_odom_bridge/odom_bridge_node.py
"""Converts FAST-LIO2 nav_msgs/Odometry (ENU world, FLU body) into
px4_msgs/VehicleOdometry (NED world, FRD body) on /fmu/in/vehicle_visual_odometry.

Design decisions, do not alter without re-reading the handover spec:
- Position + orientation only. Velocity and angular velocity are set NaN because
  EKF2_EV_CTRL=11 does not fuse them and the source twist frame is untrusted.
- Variances set NaN; EKF2_EV_NOISE_MD=1 makes PX4 use its fixed noise params.
- Timestamps use the ROS clock in microseconds; the uXRCE-DDS timesync
  (UXRCE_DDS_SYNCT=1) reconciles companion and FC clocks.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

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


class OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')
        self.declare_parameter('lio_odom_topic', '/Odometry')
        topic = self.get_parameter('lio_odom_topic').value

        # Subscribe with sensor-data-compatible QoS (FAST-LIO publishers vary).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Publish to /fmu/in with default RELIABLE QoS: compatible with the
        # agent-side subscriber. Do not change to BEST_EFFORT.
        self.pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.sub = self.create_subscription(Odometry, topic, self.cb, sub_qos)

        self.msg_count = 0
        self.create_timer(5.0, self.health)
        self.get_logger().info(f'Bridging {topic} -> /fmu/in/vehicle_visual_odometry')

    def cb(self, odom: Odometry):
        out = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = now_us
        out.timestamp_sample = now_us

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # Position: ENU -> NED is (x,y,z) -> (y, x, -z).
        p = odom.pose.pose.position
        out.position = [float(p.y), float(p.x), float(-p.z)]

        # Orientation: q_ned_frd = Q_ENU_TO_NED * q_enu_flu * Q_FLU_TO_FRD
        o = odom.pose.pose.orientation
        q_ros = np.array([o.w, o.x, o.y, o.z])
        q_px4 = q_mult(q_mult(Q_ENU_TO_NED, q_ros), Q_FLU_TO_FRD)
        q_px4 /= np.linalg.norm(q_px4)
        out.q = [float(v) for v in q_px4]

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
        if self.msg_count == 0:
            self.get_logger().warn('No LIO odometry received in last 5 s')
        self.msg_count = 0


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
