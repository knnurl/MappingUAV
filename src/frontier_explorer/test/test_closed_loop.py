# filepath: src/frontier_explorer/test/test_closed_loop.py
"""The real frontier_explorer node and the real go_to_with_avoidance node,
closed over a fake world in one process: a FollowPath action server that
moves the vehicle along the path it is given, and a map that is revealed by
raycasting from wherever the vehicle is.

What this shows: the two nodes agree on every interface, goals turn into
paths, paths into motion, motion into new map, and the mission ends. What it
does not show: anything about PX4, Aerostack2's controller, FAST-LIO or real
flight dynamics. Closed-loop debt VD-006 and VD-007 stays open.
"""
import json
import math
import threading
import time

import numpy as np
import pytest
import rclpy
from as2_msgs.action import FollowPath
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger

import fixtures
from frontier_explorer.config import Params
from frontier_explorer.explorer_node import FrontierExplorer
from frontier_explorer.grid import UNK
from map_interface.srv import QueryMap
from planner_shim.go_to_with_avoidance_node import GoToWithAvoidance

RES = 0.2
SPEED = 3.0          # m/s of the fake vehicle: fast, to keep the test short


def room_with_door(door_cells=5):
    occ = np.full((52, 18), 1, dtype=np.uint8)
    occ[3:23, 3:15] = 0
    occ[25:49, 3:15] = 0
    j0 = 9 - door_cells // 2
    occ[23:25, j0:j0 + door_cells] = 0
    return occ


TRUE = room_with_door()
BOX = fixtures.box_for(TRUE.shape, RES)
P = Params(x_min=BOX.x_min, x_max=BOX.x_max, y_min=BOX.y_min, y_max=BOX.y_max)
GRID = P.grid()
START_CELL = (13, 9)


class FakeMap(Node):
    def __init__(self, world):
        super().__init__('map_interface')
        self.world = world
        for k, v in dict(map_backend='cpu_grid', resolution=RES, max_insert_range=8.0,
                         x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0,
                         z_min=-0.5, z_max=3.0).items():
            self.declare_parameter(k, v)
        self.answer = True
        self.create_service(QueryMap, '/map_interface/query', self.on_query)

    def on_query(self, req, res):
        if not self.answer:
            time.sleep(1.0)
        occ, clr = self.world.observed()
        for p in req.points:
            i = math.floor(p.x / RES + 1e-6) - GRID.kx_min
            j = math.floor(p.y / RES + 1e-6) - GRID.ky_min
            if 0 <= i < GRID.nx and 0 <= j < GRID.ny:
                res.occupancy.append(int(occ[i, j]))
                res.clearance.append(float(clr[i, j]))
            else:
                res.occupancy.append(UNK)
                res.clearance.append(-1.0)
        return res


class FakeFence(Node):
    def __init__(self):
        super().__init__('geofence_watchdog')
        for k, v in dict(x_min=-3.0, x_max=3.0, y_min=-6.5, y_max=6.5,
                         z_min=-3.0, z_max=0.5, soft_margin=0.5).items():
            self.declare_parameter(k, v)
        self.alive = True
        self.pub = self.create_publisher(UInt8, '/geofence/state', 10)
        self.create_timer(0.25, lambda: self.alive and self.pub.publish(UInt8(data=0)))


class FakeVehicle(Node):
    """Aerostack2 FollowPath + pose + platform + LIO + bridge + battery."""

    def __init__(self, world):
        super().__init__('fake_vehicle')
        self.world = world
        x, y = GRID.center(*START_CELL)
        self.pose = [x, y, GRID.z_L]
        self.lio = 0
        self.battery = 0.9
        self.paths = []
        self.cancels = 0
        self.explorer_status = []
        self.explorer_alerts = []
        self.planner_alerts = []
        group = ReentrantCallbackGroup()
        ActionServer(self, FollowPath, '/drone0/FollowPathBehavior', self.execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=self.on_cancel, callback_group=group)
        mk = self.create_publisher
        self.pose_pub = mk(PoseStamped, '/drone0/self_localization/pose', 10)
        self.odom_pub = mk(Odometry, '/Odometry', 10)
        self.info_pub = mk(PlatformInfo, '/drone0/platform/info', 10)
        self.batt_pub = mk(BatteryState, '/drone0/sensor_measurements/battery', 10)
        self.lio_pub = mk(UInt8, '/lio_health/state', 10)
        self.bridge_pub = mk(UInt8, '/px4_odom_bridge/state', 10)
        self.create_subscription(String, '/frontier_explorer/status',
                                 lambda m: self.explorer_status.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/frontier_explorer/alert',
                                 lambda m: self.explorer_alerts.append(m.data), 10)
        self.create_subscription(String, '/go_to_with_avoidance/alert',
                                 lambda m: self.planner_alerts.append(m.data), 10)
        self.create_timer(0.05, self.tick, callback_group=group)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        x, y, z = self.pose
        self.world.reveal(GRID.world_to_cell(x, y), P.view_range_m)
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, 'earth'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation.w = 1.0
        self.pose_pub.publish(p)
        o = Odometry()
        o.header.stamp = stamp
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        self.odom_pub.publish(o)
        info = PlatformInfo(connected=True, armed=True, offboard=True)
        info.status.state = PlatformStatus.FLYING
        self.info_pub.publish(info)
        self.batt_pub.publish(BatteryState(percentage=self.battery))
        self.lio_pub.publish(UInt8(data=self.lio))
        self.bridge_pub.publish(UInt8(data=1))

    def on_cancel(self, _handle):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def execute(self, handle):
        wps = [(w.pose.position.x, w.pose.position.y, w.pose.position.z)
               for w in handle.request.path]
        self.paths.append(wps)
        for wp in wps:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    handle.canceled()
                    return FollowPath.Result(follow_path_success=False)
                d = [wp[k] - self.pose[k] for k in range(3)]
                n = math.sqrt(sum(v * v for v in d))
                step = SPEED * 0.02
                if n <= step:
                    self.pose = list(wp)
                    break
                self.pose = [self.pose[k] + d[k] / n * step for k in range(3)]
                time.sleep(0.02)
        handle.succeed()
        return FollowPath.Result(follow_path_success=True)


class Rig:
    def __init__(self, planner_overrides=None):
        self.world = fixtures.World(TRUE, RES)
        self.world.reveal(START_CELL, P.view_range_m)
        self.map = FakeMap(self.world)
        self.fence = FakeFence()
        self.vehicle = FakeVehicle(self.world)
        box = dict(x_min=BOX.x_min, x_max=BOX.x_max, y_min=BOX.y_min, y_max=BOX.y_max)
        box.update(planner_overrides or {})
        self.planner = GoToWithAvoidance(parameter_overrides=[
            Parameter(k, value=v) for k, v in box.items()] + [
            Parameter('path_check_period_s', value=0.3), Parameter('max_speed', value=1.0)])
        self.explorer = FrontierExplorer(parameter_overrides=[
            Parameter('x_min', value=BOX.x_min), Parameter('x_max', value=BOX.x_max),
            Parameter('y_min', value=BOX.y_min), Parameter('y_max', value=BOX.y_max),
            Parameter('cycle_period_s', value=0.5), Parameter('query_timeout_s', value=0.4),
            Parameter('fence_heartbeat_timeout_s', value=1.0)])
        self.helper = rclpy.create_node('closed_loop_helper')
        self.nodes = [self.map, self.fence, self.vehicle, self.planner, self.explorer,
                      self.helper]
        self.ex = MultiThreadedExecutor(num_threads=6)
        for n in self.nodes:
            self.ex.add_node(n)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def call(self, name):
        client = self.helper.create_client(Trigger, f'/frontier_explorer/{name}')
        assert client.wait_for_service(timeout_sec=5.0)
        fut = client.call_async(Trigger.Request())
        assert wait(fut.done, 5.0)
        return fut.result()

    def state(self):
        s = self.vehicle.explorer_status
        return s[-1]['state'] if s else None

    def ready(self):
        """Discovery done and the explorer has read the other nodes' parameters."""
        time.sleep(1.5)
        self.explorer.refresh_remote()
        return wait(lambda: all(t is not None for t in self.explorer.remote_t.values()), 8.0)

    def close(self):
        self.ex.shutdown()
        self.thread.join(timeout=5)
        for n in self.nodes:
            n.destroy_node()


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def make_rig():
    rclpy.init()
    rigs = []

    def _make(**kw):
        r = Rig(**kw)
        rigs.append(r)
        return r
    yield _make
    for r in rigs:
        r.close()
    rclpy.shutdown()


def test_mission_explores_both_rooms_and_ends(make_rig):
    rig = make_rig()
    assert rig.ready()
    res = rig.call('start')
    assert res.success, res.message
    assert wait(lambda: rig.state() == 'DONE', 120.0), (
        rig.state(), rig.vehicle.explorer_alerts[-5:], rig.vehicle.planner_alerts[-5:])
    last = rig.vehicle.explorer_status[-1]
    assert last['reason'] == 'COMPLETE' and last['unexplored_clusters'] == []
    assert rig.world.known[25:49, 3:15].all(), 'second room not fully seen'
    assert len(rig.vehicle.paths) >= 2
    assert rig.vehicle.pose[0] > GRID.center(24, 9)[0], 'vehicle never went through the door'
    assert not any(a.startswith('FAULT') for a in rig.vehicle.explorer_alerts)
    # Every waypoint ever flown is inside the planning box at flight altitude.
    for path in rig.vehicle.paths:
        for x, y, z in path:
            assert BOX.x_min < x < BOX.x_max and BOX.y_min < y < BOX.y_max
            assert z == pytest.approx(GRID.z_L)


def test_start_refused_when_the_planner_box_differs(make_rig):          # T-N02, C2
    rig = make_rig(planner_overrides={'x_max': BOX.x_max - 1.0})
    assert rig.ready()
    res = rig.call('start')
    assert not res.success and 'C2' in res.message
    time.sleep(1.0)
    assert rig.vehicle.paths == []


def test_lio_degraded_pauses_and_stops_the_vehicle(make_rig):           # T-N04
    rig = make_rig()
    assert rig.ready()
    assert rig.call('start').success
    assert wait(lambda: rig.vehicle.paths, 20.0)
    rig.vehicle.lio = 1
    assert wait(lambda: rig.state() == 'PAUSED', 5.0)
    assert rig.vehicle.explorer_status[-1]['reason'] == 'I3'
    n = len(rig.vehicle.paths)
    time.sleep(2.0)
    assert len(rig.vehicle.paths) == n, 'a goal was sent while LIO was degraded'
    assert not rig.call('resume').success
    rig.vehicle.lio = 0
    assert wait(lambda: rig.call('resume').success, 8.0)
    assert wait(lambda: rig.state() in ('NAVIGATING', 'SELECTING', 'DONE'), 5.0)


def test_watchdog_silence_pauses(make_rig):                              # T-N05
    rig = make_rig()
    assert rig.ready()
    assert rig.call('start').success
    assert wait(lambda: rig.vehicle.paths, 20.0)
    rig.fence.alive = False
    assert wait(lambda: rig.state() == 'PAUSED', 4.0)
    assert rig.vehicle.explorer_status[-1]['reason'] == 'I4'


def test_low_battery_ends_the_mission_at_the_start(make_rig):            # I10
    rig = make_rig()
    assert rig.ready()
    assert rig.call('start').success
    assert wait(lambda: len(rig.vehicle.paths) >= 2, 40.0)
    rig.vehicle.battery = 0.2
    assert wait(lambda: rig.state() == 'DONE', 5.0)
    assert rig.vehicle.explorer_status[-1]['reason'] == 'BATTERY'
    home = GRID.center(*START_CELL)
    assert wait(lambda: math.hypot(rig.vehicle.pose[0] - home[0],
                                   rig.vehicle.pose[1] - home[1]) < 0.3, 30.0)
