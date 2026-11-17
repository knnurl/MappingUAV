# filepath: src/lio_health_guard/lio_health_guard/guard_node.py
"""LIO health guard node (WP-D). Standalone like the geofence watchdog:
no planner/AS2 imports; it must survive their crashes.

Two sources, worst one wins:
  derived  rate, jumps and non-finite values of the odometry topic. Always on.
           The only source with upstream FAST-LIO, which publishes no health.
  native   the FAST-LIO fork's per-scan interface/LioHealth message, when
           lio_health_topic is set (config/lio_fork.yaml). Catches what the
           odometry cannot show: degenerate geometry, a filter running on IMU
           prediction alone.

On DEGRADED: alert (/lio_health/alert) + hover request
(/lio_health/hover_request) at 1 Hz until recovery; /lio_health/state at 1 Hz
(0 OK, 1 DEGRADED). Full silence of the odometry (> 1 s) belongs to the
geofence watchdog, which LANDS.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry

from .health_core import HealthMonitor, LioHealth, NativeHealthMonitor, combine


class LioHealthGuard(Node):
    def __init__(self, **node_kwargs):
        super().__init__('lio_health_guard', **node_kwargs)
        d = self.declare_parameter
        self.mon = HealthMonitor(
            d('min_rate_hz', 5.0).value, d('window_s', 1.0).value,
            d('max_speed_m_s', 3.0).value, d('recover_after_s', 2.0).value)
        odom_topic = d('lio_odom_topic', '/Odometry').value
        health_topic = d('lio_health_topic', '').value
        self.native = None
        if health_topic:
            self.native = NativeHealthMonitor(
                d('min_translation_degeneracy', 0.08).value,
                d('min_match_ratio', 0.25).value,
                d('max_residual_rms_m', 0.10).value,
                d('health_persist_s', 0.5).value, d('health_silence_s', 1.0).value,
                self.get_parameter('recover_after_s').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        if self.native is not None:
            # Imported here: the message exists only where the fork is installed.
            from interface.msg import LioHealth as LioHealthMsg
            self.create_subscription(LioHealthMsg, health_topic, self.on_health, qos)
        self.alert_pub = self.create_publisher(String, '/lio_health/alert', 10)
        self.hover_pub = self.create_publisher(Empty, '/lio_health/hover_request', 10)
        self.state_pub = self.create_publisher(UInt8, '/lio_health/state', 10)
        self.last_state = LioHealth.OK
        self.got_any = False
        self.started = self._now()
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f'LIO health guard up: derived health from {odom_topic}'
            + (f', native health from {health_topic}' if self.native else
               ' only (no native health topic configured)'))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.got_any = True
        p = msg.pose.pose.position
        self.mon.update(self._now(), p.x, p.y, p.z)
        self._transition()

    def on_health(self, msg):
        self.native.update(self._now(), msg.valid, msg.translation_degeneracy,
                           msg.match_ratio, msg.residual_rms)
        self._transition()

    def on_timer(self):
        now = self._now()
        if self.got_any:
            self.mon.tick(now)
            if self.native is not None:
                self.native.tick(now, self.started)
        state = self._transition()
        self.state_pub.publish(UInt8(data=int(state)))
        if state == LioHealth.DEGRADED:
            self.hover_pub.publish(Empty())

    def _transition(self):
        monitors = [self.mon] + ([self.native] if self.native is not None else [])
        state, reason = combine(*monitors)
        if state != self.last_state:
            self.last_state = state
            # Two call sites: rclpy refuses one call site that changes severity.
            if state == LioHealth.DEGRADED:
                text = f'LIO DEGRADED: {reason}'
                self.get_logger().warn(text)
            else:
                text = 'LIO recovered'
                self.get_logger().info(text)
            self.alert_pub.publish(String(data=text))
        return state


def main():
    rclpy.init()
    node = LioHealthGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
