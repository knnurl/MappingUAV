# filepath: src/planner_shim/planner_shim/go_to_with_avoidance_node.py
"""go_to_with_avoidance (WP-D, design B; operator decision D2, 2026-11-16).

Turns a goal pose into motion through ONE command path:
goal -> grid path on the map_interface slice -> Aerostack2 FollowPath
behaviour -> AS2 controller -> platform -> PX4 offboard. This node never
publishes to PX4 and no shortcut may be added (masterplan §6 [FIXED];
test_contract.py enforces it).

Why FollowPath and not a setpoint relay: the patched platform forwards AS2
commands only after a control mode is set following Offboard entry. AS2's own
behaviours do that; a node publishing motion references on its own never did.

Inputs
  /go_to_with_avoidance/goal           geometry_msgs/PoseStamped, odom/ENU
  /<ns>/self_localization/pose         geometry_msgs/PoseStamped (PX4-fused)
  /<ns>/platform/info                  as2_msgs/PlatformInfo
  /lio_health/state                    std_msgs/UInt8 (0 OK, 1 DEGRADED)
  /geofence/hover_request, /lio_health/hover_request,
  /go_to_with_avoidance/hover_request  std_msgs/Empty. This node is their one
      consumer: any of them cancels the active behaviour. AS2 1.1.3 has no
      hover behaviour, so "hover" is the controller holding its last reference
      after the cancel [VERIFY at Gate 6].
Outputs
  /go_to_with_avoidance/status         std_msgs/String, JSON, 2 Hz and on change
  /go_to_with_avoidance/alert          std_msgs/String, on events. Rejections
      keep the prefix "goal REJECTED" (frontier_explorer matches on it).
Clients
  /map_interface/query                 map_interface/srv/QueryMap
  /<ns>/FollowPathBehavior             as2_msgs/action/FollowPath

Status: BUILT_UNVERIFIED. Closed loop is VD-006.
"""
import json
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.action import FollowPath
from as2_msgs.msg import PlatformInfo, PlatformStatus, PoseWithID, YawMode
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String, UInt8

from map_interface.srv import QueryMap

from .path_core import (
    SliceGrid, path_clear, plan_path, points_free, remaining_waypoints,
    stop_reason,
)
from .shim_core import EnuBox, check_goal

IDLE, PLANNING, EXECUTING, REACHED, FAILED = (
    'IDLE', 'PLANNING', 'EXECUTING', 'REACHED', 'FAILED')


class GoToWithAvoidance(Node):
    def __init__(self, **node_kwargs):
        super().__init__('go_to_with_avoidance', **node_kwargs)
        d = self.declare_parameter
        # Planning box, ENU. Must sit inside the geofence and the map bounds.
        box = EnuBox(d('x_min', -4.5).value, d('x_max', 4.5).value,
                     d('y_min', -4.5).value, d('y_max', 4.5).value,
                     d('z_min', 0.2).value, d('z_max', 2.3).value)
        self.box = box
        self.inflation = float(d('obstacle_inflation', 0.4).value)
        self.grid = SliceGrid.from_box(box, float(d('resolution', 0.2).value))
        self.max_speed = float(d('max_speed', 0.5).value)
        self.snap_radius = float(d('start_snap_radius_m', 0.6).value)
        self.pose_timeout = float(d('pose_timeout_s', 0.5).value)
        self.lio_timeout = float(d('lio_heartbeat_timeout_s', 3.0).value)
        self.platform_timeout = float(d('platform_info_timeout_s', 2.0).value)
        self.map_timeout = float(d('map_query_timeout_s', 1.0).value)
        self.map_fail_max = int(d('map_fail_max', 3).value)
        self.max_replans = int(d('max_replans', 3).value)
        self.path_check_period = float(d('path_check_period_s', 1.0).value)
        # Below this the planner takes no goal and stops the active one (the
        # vehicle holds; the pilot lands). Kept under the explorer's 0.25 so its
        # fly-home goal is still accepted.
        self.battery_min = float(d('battery_min_fraction', 0.20).value)
        self.battery = None
        self.goal_frame = d('path_frame_id', 'earth').value
        ns = d('as2_namespace', 'drone0').value
        action_name = d('follow_path_action', '').value or f'/{ns}/FollowPathBehavior'
        if self.max_speed <= 0.0 or self.max_speed > 1.0:
            raise ValueError('max_speed must be in (0, 1.0] m/s (Gate 4 envelope)')

        self.state = IDLE
        self.reason = ''
        self.seq = 0                 # goal counter; stale callbacks compare it
        self.goal = None             # (x, y, z)
        self.waypoints = []          # [(x, y)] still to fly, at goal z
        self.goal_handle = None
        self.cancel_on_accept = False
        self.replans = 0
        self.blocked_count = 0
        self.map_fail_count = 0
        self.plan_deadline = None
        self.check_in_flight = False
        self.last_check = 0.0
        self.pose = None
        self.pose_t = None
        self.lio_state = None
        self.lio_t = None
        self.platform_t = None
        self.offboard = False
        self.flying = False

        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseStamped, '/go_to_with_avoidance/goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, f'/{ns}/self_localization/pose', self.on_pose, sensor)
        self.create_subscription(PlatformInfo, f'/{ns}/platform/info', self.on_platform, 10)
        self.create_subscription(UInt8, '/lio_health/state', self.on_lio, sensor)
        self.create_subscription(BatteryState, f'/{ns}/sensor_measurements/battery',
                                 self.on_battery, sensor)
        for topic in ('/geofence/hover_request', '/lio_health/hover_request',
                      '/go_to_with_avoidance/hover_request'):
            self.create_subscription(
                Empty, topic, lambda _m, t=topic: self.on_hover_request(t), 10)
        self.alert_pub = self.create_publisher(String, '/go_to_with_avoidance/alert', 10)
        self.status_pub = self.create_publisher(String, '/go_to_with_avoidance/status', 10)
        self.map_client = self.create_client(QueryMap, '/map_interface/query')
        self.path_client = ActionClient(self, FollowPath, action_name)

        self.create_timer(0.1, self.monitor)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'go_to_with_avoidance up: grid path + {action_name}, box ENU '
            f'x[{box.x_min},{box.x_max}] y[{box.y_min},{box.y_max}] '
            f'z[{box.z_min},{box.z_max}], inflation {self.inflation} m, '
            f'{self.max_speed} m/s')

    # ------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.pose = (p.x, p.y, p.z)
            self.pose_t = self.now()

    def on_platform(self, msg):
        self.platform_t = self.now()
        self.offboard = bool(msg.offboard and msg.armed)
        self.flying = msg.status.state == PlatformStatus.FLYING

    def on_battery(self, msg):
        if math.isfinite(msg.percentage):
            self.battery = msg.percentage / (100.0 if msg.percentage > 1.5 else 1.0)

    def on_lio(self, msg):
        self.lio_state = int(msg.data)
        self.lio_t = self.now()

    def gate(self):
        return stop_reason(
            self.now(), pose_t=self.pose_t, pose_timeout_s=self.pose_timeout,
            lio_state=self.lio_state, lio_t=self.lio_t, lio_timeout_s=self.lio_timeout,
            platform_t=self.platform_t, platform_timeout_s=self.platform_timeout,
            offboard=self.offboard, flying=self.flying,
            battery=self.battery, battery_min=self.battery_min)

    def on_hover_request(self, topic):
        if self.state in (PLANNING, EXECUTING):
            self.stop(f'hover requested on {topic}')

    def on_goal(self, msg):
        g = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        ok, why = check_goal(g, self.box, self.inflation)
        if not ok:
            self.alert(f'goal REJECTED: {why}')
            return
        why = self.gate()
        if why:
            # Exact text for LIO: frontier_explorer tells it apart from config drift.
            self.alert('goal REJECTED: LIO degraded' if why.startswith('LIO')
                       else f'goal REJECTED: {why}')
            return
        if not self.path_client.server_is_ready():
            self.alert('goal REJECTED: FollowPath action server not available')
            return
        self.cancel_active()
        self.seq += 1
        self.goal = g
        self.replans = 0
        self.get_logger().info(f'goal {self.seq} accepted: {g}')
        self.request_plan()

    # ---------------------------------------------------------- planning
    def query(self, points, done):
        req = QueryMap.Request()
        req.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
        fut = self.map_client.call_async(req)
        fut.add_done_callback(lambda f, n=len(points): done(f, n))

    def slice_from(self, future, n_points):
        """(occ, clr, extra_occ, extra_clr) or None if the answer is unusable."""
        try:
            res = future.result()
        except Exception as exc:        # noqa: BLE001 - service failure is data here
            self.get_logger().warn(f'map query failed: {exc}')
            return None
        if res is None or len(res.occupancy) != n_points or len(res.clearance) != n_points:
            return None
        n = self.grid.nx * self.grid.ny
        occ = np.asarray(res.occupancy, dtype=np.uint8)
        clr = np.asarray(res.clearance, dtype=np.float32)
        if not np.isin(occ, (0, 1, 2)).all():
            return None
        shape = (self.grid.nx, self.grid.ny)
        return occ[:n].reshape(shape), clr[:n].reshape(shape), occ[n:], clr[n:]

    def request_plan(self):
        self.set_state(PLANNING, '')
        if not self.map_client.service_is_ready():
            self.fail('map_interface/query not available')
            return
        seq, pose = self.seq, self.pose
        points = self.grid.slice_points(self.goal[2])
        column = []
        if abs(pose[2] - self.goal[2]) > self.grid.res / 2:
            column = self.grid.column_points(pose[0], pose[1], pose[2], self.goal[2])
        self.plan_deadline = self.now() + self.map_timeout
        self.query(points + column,
                   lambda f, n: self.on_plan_map(f, n, seq, pose, bool(column)))

    def on_plan_map(self, future, n_points, seq, pose, has_column):
        if seq != self.seq or self.state != PLANNING:
            return
        self.plan_deadline = None
        data = self.slice_from(future, n_points)
        if data is None:
            self.fail('map query gave no usable answer')
            return
        occ, clr, col_occ, col_clr = data
        if has_column and not points_free(col_occ, col_clr, self.inflation):
            self.fail('vertical move to the goal altitude is not through observed free space')
            return
        wps, why = plan_path(self.grid, occ, clr, pose[:2], self.goal[:2],
                             self.inflation, self.snap_radius)
        if wps is None:
            self.fail(why)
            return
        wps[-1] = self.goal[:2]            # finish on the goal, not the cell centre
        if has_column:
            wps = [pose[:2]] + wps         # climb or descend in place first
        self.waypoints = wps
        self.send_path(seq)

    def send_path(self, seq):
        goal = FollowPath.Goal()
        goal.header.frame_id = self.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.yaw.mode = YawMode.KEEP_YAW
        goal.max_speed = self.max_speed
        for k, (x, y) in enumerate(self.waypoints):
            wp = PoseWithID()
            wp.id = str(k)
            wp.pose.position.x, wp.pose.position.y = float(x), float(y)
            wp.pose.position.z = float(self.goal[2])
            wp.pose.orientation.w = 1.0
            goal.path.append(wp)
        self.goal_handle = None
        self.cancel_on_accept = False
        self.blocked_count = 0
        self.last_check = self.now()
        self.set_state(EXECUTING, '')
        fut = self.path_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.seq or self.state != EXECUTING or self.cancel_on_accept:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.fail('FollowPath goal was not accepted by Aerostack2')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, seq, handle))

    def on_result(self, future, seq, handle):
        if seq != self.seq or handle is not self.goal_handle or self.state != EXECUTING:
            return                          # superseded, cancelled or replanned
        wrapped = future.result()
        self.goal_handle = None
        if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and wrapped.result.follow_path_success):
            self.waypoints = []
            self.set_state(REACHED, '')
            self.alert(f'goal REACHED: {self.goal}')
        else:
            self.fail(f'FollowPath ended without success (status {wrapped.status})')

    # --------------------------------------------------------- monitoring
    def monitor(self):
        now = self.now()
        if self.state not in (PLANNING, EXECUTING):
            return
        why = self.gate()
        if why:
            self.stop(why)
            return
        if self.state == PLANNING and self.plan_deadline and now > self.plan_deadline:
            self.plan_deadline = None
            self.fail('map query timed out')
            return
        if (self.state == EXECUTING and not self.check_in_flight
                and now - self.last_check >= self.path_check_period
                and self.map_client.service_is_ready()):
            self.check_in_flight = True
            self.last_check = now
            seq = self.seq
            self.query(self.grid.slice_points(self.goal[2]),
                       lambda f, n: self.on_check_map(f, n, seq))

    def on_check_map(self, future, n_points, seq):
        self.check_in_flight = False
        if seq != self.seq or self.state != EXECUTING or self.pose is None:
            return
        data = self.slice_from(future, n_points)
        if data is None:
            self.map_fail_count += 1
            if self.map_fail_count >= self.map_fail_max:
                self.stop('map_interface stopped answering')
            return
        self.map_fail_count = 0
        occ, clr, _, _ = data
        self.waypoints = remaining_waypoints(
            self.waypoints, self.pose[:2], self.grid.res * 1.5)
        if path_clear(self.grid, occ, clr, self.pose[:2], self.waypoints,
                      self.inflation, self.inflation):
            self.blocked_count = 0
            return
        self.blocked_count += 1
        if self.blocked_count < 2:          # one bad slice is not yet a blockage
            return
        self.cancel_active()
        if self.replans >= self.max_replans:
            self.fail('path blocked and replanning exhausted')
            return
        self.replans += 1
        self.alert(f'path blocked: replanning ({self.replans}/{self.max_replans})')
        self.request_plan()

    # ------------------------------------------------------------ outputs
    def cancel_active(self):
        if self.state == EXECUTING:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            else:
                self.cancel_on_accept = True

    def stop(self, why):
        self.cancel_active()
        self.set_state(FAILED, why)
        self.alert(f'STOPPED: {why}')

    def fail(self, why):
        self.cancel_active()
        self.set_state(FAILED, why)
        self.alert(f'goal FAILED: {why}')

    def set_state(self, state, reason):
        changed = (state, reason) != (self.state, self.reason)
        self.state, self.reason = state, reason
        if changed:
            self.publish_status()

    def alert(self, text):
        self.alert_pub.publish(String(data=text))
        self.get_logger().warn(text)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason, 'seq': self.seq,
            'goal': list(self.goal) if self.goal else None,
            'waypoints_left': len(self.waypoints), 'replans': self.replans,
        })))


def main():
    rclpy.init()
    node = GoToWithAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
