# filepath: src/planner_shim/test/test_node_behavior.py
"""go_to_with_avoidance against a fake map server, a fake Aerostack2
FollowPath action server and fake pose / platform / LIO publishers, in one
process. No vehicle, no PX4: this checks wiring and the stop rules, not flight.
"""
import json
import threading
import time

import numpy as np
import pytest
import rclpy
from as2_msgs.action import FollowPath
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy import ndimage
from std_msgs.msg import Empty, String, UInt8

from map_interface.srv import QueryMap
from planner_shim.go_to_with_avoidance_node import GoToWithAvoidance
from planner_shim.path_core import FREE, OCC, UNK, SliceGrid
from planner_shim.shim_core import EnuBox

BOX = EnuBox(-3.0, 3.0, -2.0, 2.0, 0.2, 2.3)
GRID = SliceGrid.from_box(BOX, 0.2)


class Fake(Node):
    """Map (a 2D occupancy array applied at every altitude), AS2 and sensors."""

    def __init__(self):
        super().__init__('fake_world')
        self.occ = np.full((GRID.nx, GRID.ny), FREE, dtype=np.uint8)
        self.pose = [-2.0, 0.0, 1.3]
        self.lio = 0
        self.offboard = True
        self.flying = True
        self.send_lio = True
        self.goals = []
        self.cancels = 0
        self.finish = threading.Event()
        self.alerts = []
        self.status = []
        group = ReentrantCallbackGroup()
        self.create_service(QueryMap, '/map_interface/query', self.on_query,
                            callback_group=group)
        self.server = ActionServer(
            self, FollowPath, '/drone0/FollowPathBehavior', self.execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=self.on_cancel, callback_group=group)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone0/self_localization/pose', 10)
        self.info_pub = self.create_publisher(PlatformInfo, '/drone0/platform/info', 10)
        self.lio_pub = self.create_publisher(UInt8, '/lio_health/state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/go_to_with_avoidance/goal', 10)
        self.hover_pub = self.create_publisher(Empty, '/geofence/hover_request', 10)
        self.create_subscription(String, '/go_to_with_avoidance/alert',
                                 lambda m: self.alerts.append(m.data), 10)
        self.create_subscription(String, '/go_to_with_avoidance/status',
                                 lambda m: self.status.append(json.loads(m.data)), 10)
        self.create_timer(0.05, self.tick, callback_group=group)

    def tick(self):
        p = PoseStamped()
        p.header.frame_id = 'earth'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = self.pose
        self.pose_pub.publish(p)
        info = PlatformInfo()
        info.connected = True
        info.armed = True
        info.offboard = self.offboard
        info.status.state = (PlatformStatus.FLYING if self.flying
                             else PlatformStatus.LANDED)
        self.info_pub.publish(info)
        if self.send_lio:
            self.lio_pub.publish(UInt8(data=self.lio))

    def on_query(self, req, res):
        occupied = self.occ == OCC
        clr2d = (ndimage.distance_transform_edt(~occupied) * 0.2
                 if occupied.any() else np.full(self.occ.shape, 4.0))
        for p in req.points:
            c = GRID.cell(p.x, p.y)
            if c is None:
                res.occupancy.append(UNK)
                res.clearance.append(-1.0)
            else:
                res.occupancy.append(int(self.occ[c]))
                res.clearance.append(float(min(clr2d[c], 4.0)))
        return res

    def on_cancel(self, _handle):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def execute(self, handle):
        self.goals.append(handle.request)
        while rclpy.ok():
            if handle.is_cancel_requested:
                handle.canceled()
                return FollowPath.Result(follow_path_success=False)
            if self.finish.is_set():
                handle.succeed()
                return FollowPath.Result(follow_path_success=True)
            time.sleep(0.02)
        return FollowPath.Result(follow_path_success=False)

    def send_goal(self, x, y, z=1.3):
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, z
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)


@pytest.fixture
def rig():
    rclpy.init()
    fake = Fake()
    node = GoToWithAvoidance(parameter_overrides=[
        Parameter('x_min', value=BOX.x_min), Parameter('x_max', value=BOX.x_max),
        Parameter('y_min', value=BOX.y_min), Parameter('y_max', value=BOX.y_max),
        Parameter('path_check_period_s', value=0.2),
        Parameter('lio_heartbeat_timeout_s', value=1.0),
    ])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(fake)
    ex.add_node(node)
    thread = threading.Thread(target=ex.spin, daemon=True)
    thread.start()
    time.sleep(1.0)                       # discovery, first pose/platform/LIO
    yield fake, node
    fake.finish.set()
    ex.shutdown()
    thread.join(timeout=5)
    node.destroy_node()
    fake.destroy_node()
    rclpy.shutdown()


def wait(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def states(fake):
    return [s['state'] for s in fake.status]


def test_goal_becomes_a_follow_path_goal_and_reaches(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: fake.goals), fake.alerts
    goal = fake.goals[0]
    assert goal.header.frame_id == 'earth'
    assert goal.max_speed == pytest.approx(0.5)
    last = goal.path[-1].pose.position
    assert (last.x, last.y, last.z) == pytest.approx((2.0, 0.0, 1.3))
    assert wait(lambda: 'EXECUTING' in states(fake))
    fake.finish.set()
    assert wait(lambda: fake.status and fake.status[-1]['state'] == 'REACHED')
    assert any(a.startswith('goal REACHED') for a in fake.alerts)


def test_path_goes_round_a_wall(rig):
    fake, _ = rig
    i_wall = GRID.cell(0.0, 0.0)[0]
    fake.occ[i_wall, 0:GRID.ny - 6] = OCC          # wall with a gap at +y
    fake.send_goal(2.0, -1.0)
    assert wait(lambda: fake.goals), fake.alerts
    path = [(p.pose.position.x, p.pose.position.y) for p in fake.goals[0].path]
    assert len(path) >= 2
    assert max(y for _, y in path) > 0.5            # went up to the gap


def test_goal_outside_the_box_is_rejected(rig):
    fake, _ = rig
    fake.send_goal(9.0, 0.0)
    assert wait(lambda: any(a.startswith('goal REJECTED') for a in fake.alerts))
    time.sleep(0.3)
    assert fake.goals == []


def test_goal_in_unknown_space_fails_without_motion(rig):
    fake, _ = rig
    fake.occ[GRID.cell(1.0, 0.0)[0]:, :] = UNK
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: any(a.startswith('goal FAILED') for a in fake.alerts))
    assert fake.goals == []


def test_lio_degraded_rejects_with_the_exact_text(rig):
    fake, _ = rig
    fake.lio = 1
    time.sleep(0.3)
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: 'goal REJECTED: LIO degraded' in fake.alerts)
    assert fake.goals == []


def test_lio_silence_stops_a_running_goal(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: fake.goals)
    fake.send_lio = False
    assert wait(lambda: fake.cancels >= 1, timeout=4.0)
    assert wait(lambda: fake.status[-1]['state'] == 'FAILED')
    assert 'LIO' in fake.status[-1]['reason']


def test_hover_request_cancels_the_behaviour(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: fake.goals and 'EXECUTING' in states(fake))
    time.sleep(0.2)
    fake.hover_pub.publish(Empty())
    assert wait(lambda: fake.cancels >= 1)
    assert wait(lambda: fake.status[-1]['state'] == 'FAILED')
    assert 'hover requested' in fake.status[-1]['reason']


def test_leaving_offboard_cancels_and_later_goals_are_refused(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: fake.goals)
    time.sleep(0.2)
    fake.offboard = False
    assert wait(lambda: fake.cancels >= 1)
    n = len(fake.goals)
    fake.send_goal(1.0, 0.0)
    assert wait(lambda: any('not in Offboard' in a for a in fake.alerts))
    assert len(fake.goals) == n


def test_obstacle_appearing_on_the_path_triggers_a_replan(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: len(fake.goals) == 1)
    time.sleep(0.3)
    i_wall = GRID.cell(0.0, 0.0)[0]
    fake.occ[i_wall, 0:GRID.ny - 6] = OCC          # blocks the straight line
    assert wait(lambda: len(fake.goals) == 2, timeout=6.0), fake.alerts
    assert fake.cancels >= 1
    assert any(a.startswith('path blocked') for a in fake.alerts)
    path = [(p.pose.position.x, p.pose.position.y) for p in fake.goals[1].path]
    assert max(y for _, y in path) > 0.5


def test_a_new_goal_replaces_the_running_one(rig):
    fake, _ = rig
    fake.send_goal(2.0, 0.0)
    assert wait(lambda: len(fake.goals) == 1)
    time.sleep(0.2)
    fake.send_goal(0.0, 1.0)
    assert wait(lambda: len(fake.goals) == 2)
    assert fake.cancels >= 1
    last = fake.goals[1].path[-1].pose.position
    assert (last.x, last.y) == pytest.approx((0.0, 1.0))


def test_altitude_change_goes_up_first_through_free_cells(rig):
    fake, _ = rig
    fake.pose[2] = 0.7
    time.sleep(0.2)
    fake.send_goal(2.0, 0.0, 1.3)
    assert wait(lambda: fake.goals), fake.alerts
    first = fake.goals[0].path[0].pose.position
    assert (first.x, first.y, first.z) == pytest.approx((-2.0, 0.0, 1.3))
