# filepath: src/sim_bringup/sim_bringup/fake_lidar.py
"""Stands in for the Mid-360 driver AND FAST-LIO in the closed-loop rig:
publishes /Odometry and /cloud_registered at 10 Hz in FAST-LIO's conventions
(world = start pose, x forward, z up; cloud in the world frame), from the fake
PX4's true pose and a raycast into a box world.

Because it bypasses FAST-LIO and EKF2, the rig says NOTHING about estimation.
Fault hooks: /sim/lio_fault (std_msgs/String): 'silent', 'jump', 'ok'.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .world import WORLDS, ray_directions, scan


class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        name = self.declare_parameter('world', 'two_rooms').value
        self.world = WORLDS[name]()
        odom_topic = self.declare_parameter('lio_odom_topic', '/Odometry').value
        cloud_topic = self.declare_parameter('lio_cloud_topic', '/cloud_registered').value
        health_topic = self.declare_parameter('lio_health_topic', '').value
        self.dirs = ray_directions()
        self.pose = None                  # (x, y, z, yaw) in the LIO frame
        self.fault = 'ok'
        self.offset = np.zeros(3)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.on_truth, qos_profile_sensor_data)
        self.create_subscription(String, '/sim/lio_fault', self.on_fault, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.health_pub = None
        if health_topic:                  # the fork's health message, when it is installed
            from interface.msg import LioHealth
            self.health_msg = LioHealth
            self.health_pub = self.create_publisher(LioHealth, health_topic, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'fake LiDAR + LIO up, world "{name}"')

    def on_fault(self, msg):
        # 'silent', 'jump', 'ok', and with the fork's naming 'degenerate'
        # (odometry keeps coming, health reports an unconstrained direction).
        self.fault = msg.data
        if msg.data == 'jump':
            self.offset = np.array([3.0, 0.0, 0.0])
        elif msg.data == 'ok':
            self.offset = np.zeros(3)

    def on_truth(self, msg):
        n, e, d = msg.position
        yaw_ned = math.atan2(2.0 * (msg.q[0] * msg.q[3] + msg.q[1] * msg.q[2]),
                             1.0 - 2.0 * (msg.q[2] ** 2 + msg.q[3] ** 2))
        self.pose = (float(e), float(n), float(-d), math.pi / 2 - yaw_ned)

    def tick(self):
        if self.pose is None or self.fault == 'silent':
            return
        x, y, z, yaw = self.pose
        stamp = self.get_clock().now().to_msg()
        pts = scan(self.world, (x, y, z), yaw, self.dirs) + self.offset.astype(np.float32)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'camera_init', 'body'
        od.pose.pose.position.x = x + float(self.offset[0])
        od.pose.pose.position.y, od.pose.pose.position.z = y, z
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.odom_pub.publish(od)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, 'camera_init'
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                        for k, n in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, 12 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self.cloud_pub.publish(cloud)
        if self.health_pub is not None:
            h = self.health_msg()
            h.header = od.header
            h.valid = True
            h.points, h.matched = len(pts), int(0.56 * len(pts))
            h.match_ratio, h.residual_rms, h.iterations = 0.56, 0.042, 2
            h.translation_degeneracy = 0.02 if self.fault == 'degenerate' else 0.40
            h.rotation_degeneracy = 0.5
            h.weakest_translation_dir.x = 1.0
            self.health_pub.publish(h)


def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
