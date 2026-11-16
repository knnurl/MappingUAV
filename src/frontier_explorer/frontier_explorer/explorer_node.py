# filepath: src/frontier_explorer/frontier_explorer/explorer_node.py
"""frontier_explorer ROS adapter (spec §3.2, §4). All decisions are made in
the pure modules; this file only moves data between ROS and them.

The node's ONLY motion-related output is a PoseStamped goal on
/go_to_with_avoidance/goal (plus a hover request). It never arms, takes off,
lands, changes mode or talks to PX4 (SR-2; test_contract.py and the node
test's graph check enforce it).

Status: BUILT_UNVERIFIED (closed loop is VD-007).
"""
import dataclasses
import json
import math
import time

import numpy as np
import rclpy
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String, UInt8
from std_srvs.srv import Trigger

from map_interface.srv import QueryMap

from . import config, fsm
from .grid import chunks, validate_response

BOX_KEYS = ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max']
REMOTE = {
    'shim': BOX_KEYS + ['obstacle_inflation'],
    'map': BOX_KEYS + ['map_backend', 'resolution', 'max_insert_range'],
    'fence': BOX_KEYS + ['soft_margin'],
}
CHECKS = {'shim': config.check_shim, 'map': config.check_map, 'fence': config.check_fence}


def param_value(v):
    return {ParameterType.PARAMETER_BOOL: v.bool_value,
            ParameterType.PARAMETER_INTEGER: v.integer_value,
            ParameterType.PARAMETER_DOUBLE: v.double_value,
            ParameterType.PARAMETER_STRING: v.string_value}.get(v.type)


def yaw_only(q):
    """Quaternion with roll and pitch removed (spec §7.15)."""
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FrontierExplorer(Node):
    def __init__(self, **node_kwargs):
        super().__init__('frontier_explorer', **node_kwargs)
        defaults = config.Params()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in dataclasses.fields(config.Params)}
        self.p = p = config.Params(**values)
        self.config_errors = config.validate(p)
        self.mission = fsm.Mission(p)
        self.frames = fsm.FrameMonitor(p)
        self.grid = None
        self.points = []
        if not self.config_errors:
            self.grid = p.grid()
            self.points = self.grid.query_points()

        self.inp = fsm.Inputs(now=self.now())
        self.yaw = (0.0, 1.0)
        self.alerts_in = []
        self.planner_seq = 0
        self.planner_seq_at_publish = 0
        self.cycle = None             # in-flight cycle bookkeeping
        self.skipped_cycles = 0
        self.last_query_ms = 0.0
        self.remote = {k: None for k in REMOTE}      # name -> dict or None
        self.remote_t = {k: None for k in REMOTE}

        ns = p.as2_namespace
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5)
        sub = self.create_subscription
        sub(PoseStamped, f'/{ns}/self_localization/pose', self.on_pose, sensor)
        sub(PlatformInfo, f'/{ns}/platform/info', self.on_platform, 10)
        sub(BatteryState, f'/{ns}/sensor_measurements/battery', self.on_battery, sensor)
        sub(UInt8, '/lio_health/state', self.on_lio, sensor)
        sub(UInt8, '/geofence/state', self.on_fence, sensor)
        sub(UInt8, '/px4_odom_bridge/state', self.on_bridge, sensor)
        sub(String, '/go_to_with_avoidance/alert', lambda m: self.alerts_in.append(m.data), 10)
        sub(String, '/go_to_with_avoidance/status', self.on_planner_status, 10)
        sub(Odometry, '/Odometry', self.on_lio_odom, sensor)

        self.goal_pub = self.create_publisher(PoseStamped, '/go_to_with_avoidance/goal', 10)
        self.hover_pub = self.create_publisher(Empty, '/go_to_with_avoidance/hover_request', 10)
        self.state_pub = self.create_publisher(UInt8, '/frontier_explorer/state', 10)
        self.status_pub = self.create_publisher(String, '/frontier_explorer/status', 10)
        self.alert_pub = self.create_publisher(String, '/frontier_explorer/alert', 10)

        self.map_client = self.create_client(QueryMap, '/map_interface/query')
        self.param_clients = {
            'shim': self.create_client(GetParameters, f'{p.shim_node}/get_parameters'),
            'map': self.create_client(GetParameters, f'{p.map_node}/get_parameters'),
            'fence': self.create_client(GetParameters, f'{p.fence_node}/get_parameters'),
        }
        for name, handler in (('start', self.srv_start), ('pause', self.srv_pause),
                              ('resume', self.srv_resume), ('stop', self.srv_stop)):
            self.create_service(Trigger, f'/frontier_explorer/{name}', handler)

        if self.config_errors:
            self.do(self.mission.fault(self.inp, 'I8', 'CONFIG ' + '; '.join(self.config_errors)))
            self.get_logger().fatal('configuration invalid: ' + '; '.join(self.config_errors))
        else:
            self.create_timer(p.cycle_period_s, self.guarded(self.on_cycle_timer))
            self.create_timer(5.0, self.refresh_remote)
            self.refresh_remote()
        self.create_timer(0.1, self.guarded(self.monitor))
        self.create_timer(1.0, self.publish_status)

    # ---------------------------------------------------------- plumbing
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def guarded(self, fn):
        """T21: an exception in planning or a callback is FAULT(I8), not a crash."""
        def wrapper(*args):
            try:
                return fn(*args)
            except Exception as exc:        # noqa: BLE001
                self.get_logger().error(f'internal error: {exc!r}')
                self.inp.now = self.now()
                self.do(self.mission.fault(self.inp, 'I8', repr(exc)))
        return wrapper

    def do(self, actions):
        for a in actions:
            if a[0] == 'goal':
                self.publish_goal(a[1])
            elif a[0] == 'hover':
                self.hover_pub.publish(Empty())
            elif a[0] == 'alert':
                self.alert_pub.publish(String(data=a[1]))
                self.get_logger().info(a[1])

    def publish_goal(self, xyz):
        msg = PoseStamped()
        msg.header.frame_id = self.p.goal_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        msg.pose.orientation.z, msg.pose.orientation.w = self.yaw
        self.planner_seq_at_publish = self.planner_seq
        self.goal_pub.publish(msg)

    # ------------------------------------------------------------ inputs
    def on_pose(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if not all(math.isfinite(v) for v in pos):
            return
        self.inp.pose, self.inp.pose_t = pos, self.now()
        self.yaw = yaw_only(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames.on_pose(stamp, pos)
        self.inp.frame_diverged, self.inp.frame_valid = self.frames.diverged, self.frames.valid

    def on_lio_odom(self, msg):
        p = msg.pose.pose.position
        self.frames.on_lio(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                           (p.x, p.y, p.z))

    def on_platform(self, msg):
        i = self.inp
        i.platform_t = self.now()
        i.connected, i.armed, i.offboard = msg.connected, msg.armed, msg.offboard
        i.flying = msg.status.state == PlatformStatus.FLYING

    def on_battery(self, msg):
        if math.isfinite(msg.percentage):
            # AS2/PX4 report 0..1; some drivers report 0..100.
            self.inp.battery = msg.percentage / (100.0 if msg.percentage > 1.5 else 1.0)
            self.inp.battery_t = self.now()

    def on_lio(self, msg):
        self.inp.lio_state, self.inp.lio_t = int(msg.data), self.now()

    def on_fence(self, msg):
        self.inp.fence_state, self.inp.fence_t = int(msg.data), self.now()

    def on_bridge(self, msg):
        self.inp.bridge_state, self.inp.bridge_t = int(msg.data), self.now()

    def on_planner_status(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.planner_seq = int(s.get('seq', 0))
        if s.get('state') == 'FAILED' and self.planner_seq > self.planner_seq_at_publish:
            self.inp.now = self.now()
            self.planner_seq_at_publish = self.planner_seq    # handle once
            self.do(self.mission.on_planner_failed(self.inp, s.get('reason', '')))

    # ------------------------------------------- remote config (C2-C4, C7)
    def refresh_remote(self):
        for name, client in self.param_clients.items():
            if not client.service_is_ready():
                continue
            req = GetParameters.Request(names=REMOTE[name])
            client.call_async(req).add_done_callback(
                lambda f, n=name: self.on_remote(n, f))

    def on_remote(self, name, future):
        try:
            values = future.result().values
        except Exception:                   # noqa: BLE001
            return
        self.remote[name] = {k: param_value(v) for k, v in zip(REMOTE[name], values)}
        self.remote_t[name] = self.now()

    def remote_errors(self):
        errors = list(self.config_errors)
        now = self.now()
        for name in REMOTE:
            t = self.remote_t[name]
            if t is None or now - t > 15.0:
                errors.append(f'C7: no parameters from the {name} node')
            else:
                errors.extend(CHECKS[name](self.p, self.remote[name]))
        return errors

    # ------------------------------------------------------------ services
    def reply(self, res, result):
        ok, message, actions = result
        self.do(actions)
        res.success, res.message = ok, message
        return res

    def srv_start(self, _req, res):
        self.inp.now = self.now()
        self.inp.map_available = self.map_client.service_is_ready()
        return self.reply(res, self.mission.start(self.inp, self.remote_errors()))

    def srv_pause(self, _req, res):
        self.inp.now = self.now()
        return self.reply(res, self.mission.pause(self.inp))

    def srv_resume(self, _req, res):
        self.inp.now = self.now()
        return self.reply(res, self.mission.resume(self.inp, self.remote_errors()))

    def srv_stop(self, _req, res):
        self.inp.now = self.now()
        return self.reply(res, self.mission.stop(self.inp))

    # ------------------------------------------------------- 10 Hz monitor
    def monitor(self):
        self.inp.now = self.now()
        if self.cycle and self.inp.now > self.cycle['deadline']:
            self.cycle = None               # late answers are discarded by id
            self.do(self.mission.on_cycle(self.inp, fsm.Cycle(ok=False)))
        alerts, self.alerts_in = self.alerts_in, []
        self.do(self.mission.tick(self.inp, alerts))

    # ------------------------------------------------------ planning cycle
    def on_cycle_timer(self):
        if not self.mission.wants_cycle():
            return
        if self.cycle is not None:
            self.skipped_cycles += 1
            return
        if not self.map_client.service_is_ready():
            self.inp.now = self.now()
            self.do(self.mission.on_cycle(self.inp, fsm.Cycle(ok=False)))
            return
        parts = chunks(self.points, self.p.max_points_per_request)
        self.cycle = {'id': time.monotonic(), 't0': time.monotonic(),
                      'deadline': self.now() + self.p.query_timeout_s,
                      'occ': [None] * len(parts), 'clr': [None] * len(parts)}
        for k, part in enumerate(parts):
            req = QueryMap.Request(points=[Point(x=x, y=y, z=z) for x, y, z in part])
            self.map_client.call_async(req).add_done_callback(self.guarded(
                lambda f, k=k, n=len(part), cid=self.cycle['id']: self.on_chunk(f, k, n, cid)))

    def on_chunk(self, future, k, n, cid):
        c = self.cycle
        if c is None or c['id'] != cid:
            return
        res = future.result()
        if res is None or not validate_response(n, res.occupancy, res.clearance):
            self.cycle = None
            self.inp.now = self.now()
            self.do(self.mission.on_cycle(self.inp, fsm.Cycle(ok=False)))
            return
        c['occ'][k], c['clr'][k] = res.occupancy, res.clearance
        if any(part is None for part in c['occ']):
            return
        self.cycle = None
        self.last_query_ms = (time.monotonic() - c['t0']) * 1e3
        shape = (self.grid.nx, self.grid.ny)
        occ = np.concatenate([np.asarray(o, dtype=np.uint8) for o in c['occ']]).reshape(shape)
        clr = np.concatenate([np.asarray(o, dtype=np.float32) for o in c['clr']]).reshape(shape)
        self.inp.now = self.now()
        if self.inp.pose is None:
            return
        t0 = time.monotonic()
        cyc = fsm.run_cycle(self.p, self.grid, occ, clr, self.inp.pose[:2],
                            self.mission.blacklist.active(self.inp.now),
                            self.mission.goal_cell)
        cyc.plan_ms = (time.monotonic() - t0) * 1e3
        if cyc.plan_ms > self.p.max_plan_time_s * 1e3:
            self.get_logger().warn(f'planning took {cyc.plan_ms:.0f} ms')
        self.do(self.mission.on_cycle(self.inp, cyc))

    # -------------------------------------------------------------- status
    def publish_status(self):
        now = self.now()
        self.state_pub.publish(UInt8(data=int(self.mission.state)))
        self.status_pub.publish(String(data=json.dumps(self.mission.status(now, {
            'skipped_cycles': self.skipped_cycles,
            'last_query_ms': round(self.last_query_ms, 1),
            'battery': self.inp.battery,
            'frame_diff_m': self.frames.last_diff,
        }))))


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
