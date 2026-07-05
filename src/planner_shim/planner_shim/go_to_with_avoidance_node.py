# filepath: src/planner_shim/planner_shim/go_to_with_avoidance_node.py
"""go_to_with_avoidance wrapper node (WP-D skeleton, BUILT_UNVERIFIED).

One command path [FIXED, masterplan §6]: behavior -> EGO planner -> AS2
motion pipeline -> PX4 offboard. No planner-direct-to-PX4 shortcut exists in
this node, and none may be added.

Skeleton scope (closed loop is untestable without sim/vehicle — VD entry):
- Accepts a goal on /go_to_with_avoidance/goal (geometry_msgs/PoseStamped).
- Validates it (shim_core.check_goal) against the planning box; rejects
  loudly on failure.
- Forwards accepted goals to the EGO planner goal topic [VERIFY at
  integration: exact topic of the pinned ego-planner-swarm ros2_version].
- Relays EGO position commands to the AS2 motion reference topic, gated by
  trajectory validity (shim_core sliding checks) and lio_health state:
  DEGRADED => stop relaying, request hover (behavior freezes, watchdog and
  AS2 keep authority).
Topics are parameters so integration can rewire without code changes.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty, UInt8

from .shim_core import EnuBox, check_goal

LIO_DEGRADED = 1


class GoToWithAvoidance(Node):
    def __init__(self):
        super().__init__('go_to_with_avoidance')
        # Planning box: derive from geofence, shrink, keep in ENU [FIXED].
        self.declare_parameter('x_min', -4.5)
        self.declare_parameter('x_max', 4.5)
        self.declare_parameter('y_min', -4.5)
        self.declare_parameter('y_max', 4.5)
        self.declare_parameter('z_min', 0.2)
        self.declare_parameter('z_max', 2.3)
        self.declare_parameter('obstacle_inflation', 0.4)
        self.declare_parameter('ego_goal_topic', '/goal')          # [VERIFY]
        self.declare_parameter('ego_cmd_topic', '/position_cmd')   # [VERIFY]
        self.declare_parameter('as2_motion_ref_topic',
                               '/drone0/motion_reference/pose')    # [VERIFY]

        p = self.get_parameter
        self.box = EnuBox(p('x_min').value, p('x_max').value,
                          p('y_min').value, p('y_max').value,
                          p('z_min').value, p('z_max').value)
        self.inflation = p('obstacle_inflation').value
        self.lio_degraded = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseStamped, '/go_to_with_avoidance/goal',
                                 self.on_goal, 10)
        self.create_subscription(UInt8, '/lio_health/state',
                                 self.on_lio_state, qos)
        self.create_subscription(PoseStamped, p('ego_cmd_topic').value,
                                 self.on_ego_cmd, qos)
        self.ego_goal_pub = self.create_publisher(
            PoseStamped, p('ego_goal_topic').value, 10)
        self.motion_ref_pub = self.create_publisher(
            PoseStamped, p('as2_motion_ref_topic').value, 10)
        self.alert_pub = self.create_publisher(
            String, '/go_to_with_avoidance/alert', 10)
        self.hover_pub = self.create_publisher(
            Empty, '/go_to_with_avoidance/hover_request', 10)
        self.get_logger().info(
            f'go_to_with_avoidance up; planning box ENU '
            f'x[{self.box.x_min},{self.box.x_max}] '
            f'y[{self.box.y_min},{self.box.y_max}] '
            f'z[{self.box.z_min},{self.box.z_max}]')

    def on_lio_state(self, msg: UInt8):
        was = self.lio_degraded
        self.lio_degraded = (msg.data == LIO_DEGRADED)
        if self.lio_degraded and not was:
            self.alert_pub.publish(String(
                data='LIO degraded: freezing avoidance relay, requesting hover'))
            self.hover_pub.publish(Empty())

    def on_goal(self, msg: PoseStamped):
        g = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        ok, why = check_goal(g, self.box, self.inflation)
        if not ok:
            self.alert_pub.publish(String(data=f'goal REJECTED: {why}'))
            self.get_logger().error(f'goal rejected: {why}')
            return
        if self.lio_degraded:
            self.alert_pub.publish(String(data='goal REJECTED: LIO degraded'))
            return
        self.ego_goal_pub.publish(msg)
        self.get_logger().info(f'goal accepted -> EGO: {g}')

    def on_ego_cmd(self, msg: PoseStamped):
        if self.lio_degraded:
            self.hover_pub.publish(Empty())
            return
        pos = msg.pose.position
        if not self.box.contains(pos.x, pos.y, pos.z):
            self.alert_pub.publish(String(
                data='EGO command left planning box: dropped, hover requested'))
            self.hover_pub.publish(Empty())
            return
        self.motion_ref_pub.publish(msg)


def main():
    rclpy.init()
    node = GoToWithAvoidance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
