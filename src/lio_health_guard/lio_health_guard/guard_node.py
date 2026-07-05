# filepath: src/lio_health_guard/lio_health_guard/guard_node.py
"""LIO degeneracy guard node (WP-D). Standalone like the geofence watchdog:
no planner/AS2 imports; it must survive their crashes.

On DEGRADED: publish alert (/lio_health/alert) + hover request
(/lio_health/hover_request) at 1 Hz until recovery. The hard silence case
(>1 s no odometry at all) belongs to the geofence watchdog, which LANDS.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry

from .health_core import HealthMonitor, LioHealth


class LioHealthGuard(Node):
    def __init__(self):
        super().__init__('lio_health_guard')
        self.declare_parameter('min_rate_hz', 5.0)
        self.declare_parameter('window_s', 1.0)
        self.declare_parameter('max_speed_m_s', 3.0)
        self.declare_parameter('recover_after_s', 2.0)

        self.mon = HealthMonitor(
            self.get_parameter('min_rate_hz').value,
            self.get_parameter('window_s').value,
            self.get_parameter('max_speed_m_s').value,
            self.get_parameter('recover_after_s').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos)
        self.alert_pub = self.create_publisher(String, '/lio_health/alert', 10)
        self.hover_pub = self.create_publisher(Empty, '/lio_health/hover_request', 10)
        self.state_pub = self.create_publisher(UInt8, '/lio_health/state', 10)
        self.last_state = LioHealth.OK
        self.got_any = False
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info('LIO health guard up (derived health; FAST-LIO '
                               'publishes no native health topic)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.got_any = True
        p = msg.pose.pose.position
        state = self.mon.update(self._now(), p.x, p.y, p.z)
        self._transition(state)

    def on_timer(self):
        if self.got_any:
            state = self.mon.tick(self._now())
            self._transition(state)
        self.state_pub.publish(UInt8(data=int(self.mon.state)))
        if self.mon.state == LioHealth.DEGRADED:
            self.hover_pub.publish(Empty())

    def _transition(self, state):
        if state != self.last_state:
            msg = (f'LIO {state.name}: {self.mon.reason}' if
                   state == LioHealth.DEGRADED else 'LIO recovered')
            self.alert_pub.publish(String(data=msg))
            log = (self.get_logger().warn if state == LioHealth.DEGRADED
                   else self.get_logger().info)
            log(msg)
            self.last_state = state


def main():
    rclpy.init()
    node = LioHealthGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
