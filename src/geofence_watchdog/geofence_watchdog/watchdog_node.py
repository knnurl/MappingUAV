# filepath: src/geofence_watchdog/geofence_watchdog/watchdog_node.py
"""Geofence watchdog node (WP-E). Standalone by design [FIXED]: zero imports
from Aerostack2, planners, or the map server — it must survive their crashes.

Inputs:
  /fmu/out/vehicle_local_position  px4_msgs/VehicleLocalPosition, BEST_EFFORT.
      Deliberately NOT the LIO topic: the watchdog polices the estimate the
      controller acts on.
  /Odometry                        nav_msgs/Odometry, BEST_EFFORT.
      Liveness only (LIO silence > lio_timeout_s is a hard failure); field
      values are never used for fencing.

Outputs:
  /fmu/in/vehicle_command          px4_msgs/VehicleCommand, RELIABLE.
      VEHICLE_CMD_NAV_LAND on hard action, re-sent every cmd_resend_period_s
      while the hard condition persists.
  /geofence/alert                  std_msgs/String, on state changes.
  /geofence/hover_request          std_msgs/Empty, on soft breach. The
      behavior layer (WP-B) subscribes to this later; publishing to nothing
      is fine — the hard action never depends on a subscriber existing.
  /geofence/state                  std_msgs/UInt8 heartbeat at 2 Hz
      (FenceState value). Consumers treat heartbeat silence as watchdog death.

HARD latches until node restart: once a land has been commanded, the vehicle
is landing; automatic un-latching could fight the pilot or re-enable autonomy.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleCommand

from .geofence_core import (
    GeofenceBox, BreachStateMachine, TimeoutMonitor, Action, decide,
)


class GeofenceWatchdog(Node):
    def __init__(self):
        super().__init__('geofence_watchdog')

        # Box in local NED [FIXED frame]; defaults match the Gate 4 tethered
        # test volume (3 x 3 x 2.5 m). Override via drone_bringup geofence.yaml.
        self.declare_parameter('x_min', -1.5)
        self.declare_parameter('x_max', 1.5)
        self.declare_parameter('y_min', -1.5)
        self.declare_parameter('y_max', 1.5)
        self.declare_parameter('z_min', -2.5)   # ceiling (NED: up is negative)
        self.declare_parameter('z_max', 0.3)    # floor bound, ground tolerance
        self.declare_parameter('soft_margin', 0.5)
        self.declare_parameter('hysteresis', 0.2)
        self.declare_parameter('lio_timeout_s', 1.0)
        self.declare_parameter('estimate_timeout_s', 1.0)
        # Grace before 'stream never seen' counts as silence. None-equivalent
        # (<0) disables, so a bench watchdog without the pipeline stays quiet.
        self.declare_parameter('require_streams_after_s', -1.0)
        self.declare_parameter('cmd_resend_period_s', 1.0)

        def p(n):
            return self.get_parameter(n).value
        box = GeofenceBox(p('x_min'), p('x_max'), p('y_min'), p('y_max'),
                          p('z_min'), p('z_max'), p('soft_margin'))
        self.fence = BreachStateMachine(box, p('hysteresis'))
        require = p('require_streams_after_s')
        require = None if require < 0 else require
        self.lio_mon = TimeoutMonitor(p('lio_timeout_s'), require)
        self.est_mon = TimeoutMonitor(p('estimate_timeout_s'), require)
        self.cmd_resend_period = p('cmd_resend_period_s')

        now = self._now_s()
        self.lio_mon.start(now)
        self.est_mon.start(now)

        self.est_valid = True   # until first message says otherwise
        self.pos = None         # (x, y, z) NED
        self.last_action = Action.NONE
        self.last_land_sent_s = None

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, best_effort)
        self.create_subscription(
            Odometry, '/Odometry', self._on_lio, best_effort)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.alert_pub = self.create_publisher(String, '/geofence/alert', 10)
        self.hover_pub = self.create_publisher(Empty, '/geofence/hover_request', 10)
        self.state_pub = self.create_publisher(UInt8, '/geofence/state', 10)

        # 10 Hz evaluation; 2 Hz heartbeat [FIXED].
        self.create_timer(0.1, self._evaluate)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(
            f'Geofence active, NED box x[{box.x_min},{box.x_max}] '
            f'y[{box.y_min},{box.y_max}] z[{box.z_min},{box.z_max}] '
            f'margin {box.soft_margin} hyst {self.fence.hysteresis}')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_local_position(self, msg: VehicleLocalPosition):
        self.est_mon.beat(self._now_s())
        self.est_valid = bool(msg.xy_valid and msg.z_valid)
        self.pos = (float(msg.x), float(msg.y), float(msg.z))

    def _on_lio(self, _msg: Odometry):
        self.lio_mon.beat(self._now_s())

    def _evaluate(self):
        now = self._now_s()
        if self.pos is not None:
            fence_state = self.fence.update(*self.pos)
        else:
            fence_state = self.fence.state
        action = decide(fence_state, self.est_valid,
                        self.lio_mon.expired(now), self.est_mon.expired(now))

        if action != self.last_action:
            reason = (f'fence={fence_state.name} est_valid={self.est_valid} '
                      f'lio_silent={self.lio_mon.expired(now)} '
                      f'est_silent={self.est_mon.expired(now)} pos={self.pos}')
            self.alert_pub.publish(String(data=f'{action.name}: {reason}'))
            log = (self.get_logger().error if action == Action.LAND
                   else self.get_logger().warn if action == Action.ALERT_HOVER
                   else self.get_logger().info)
            log(f'geofence action {action.name}: {reason}')
            self.last_action = action

        if action == Action.ALERT_HOVER:
            self.hover_pub.publish(Empty())
        elif action == Action.LAND:
            if (self.last_land_sent_s is None
                    or now - self.last_land_sent_s >= self.cmd_resend_period):
                self._send_land(now)
                self.last_land_sent_s = now

    def _send_land(self, now_s):
        cmd = VehicleCommand()
        cmd.timestamp = int(now_s * 1e6)
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)

    def _heartbeat(self):
        self.state_pub.publish(UInt8(data=int(self.fence.state)))


def main():
    rclpy.init()
    node = GeofenceWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
