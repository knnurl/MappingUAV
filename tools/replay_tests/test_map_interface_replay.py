# filepath: tools/replay_tests/test_map_interface_replay.py
"""WP-C headless replay harness: drives a real map_server_node process with
synthetic clouds and asserts on its outputs (occupancy at known obstacles,
free space, query latency budget, no crash over the run).

Run:  python3 -m pytest tools/replay_tests/ -q   (workspace sourced)
Replace the synthetic feed with rosbag playback once Gate 2/3 bags exist —
assertions stay the same (that is the point of this harness).
"""
import os
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(__file__))
from synthetic_scene import Scene, scene_points, visible_slice, circular_trajectory  # noqa: E402

from map_interface.srv import QueryMap  # noqa: E402

import struct  # noqa: E402

OCC_FREE, OCC_OCCUPIED, OCC_UNKNOWN = 0, 1, 2


def make_cloud(points, stamp, frame='odom'):
    msg = PointCloud2()
    msg.header = Header(frame_id=frame)
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('fff', *p) for p in points)
    return msg


class Feeder(Node):
    def __init__(self):
        super().__init__('replay_feeder')
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered', 5)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 5)
        self.client = self.create_client(QueryMap, '/map_interface/query')

    def publish_pose_and_cloud(self, pose, cloud_pts):
        od = Odometry()
        od.header.frame_id = 'odom'
        od.header.stamp = self.get_clock().now().to_msg()
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = pose
        od.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(od)
        # The map node pairs each cloud with the LAST odom it saw; give the
        # pose time to land first or rays are cast from a stale origin and
        # carve through obstacles (cross-topic ordering is not guaranteed).
        time.sleep(0.05)
        self.cloud_pub.publish(make_cloud(cloud_pts, od.header.stamp))

    def query(self, points, timeout=5.0):
        req = QueryMap.Request()
        req.points = [Point(x=float(x), y=float(y), z=float(z)) for (x, y, z) in points]
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        assert fut.done(), 'query service did not answer in time'
        return fut.result()


@pytest.fixture(scope='module')
def stack():
    env = os.environ.copy()
    # Launch the node binary DIRECTLY: killing a `ros2 run` wrapper leaks the
    # node child, and orphaned servers keep answering /map_interface/query
    # with stale maps (cost half a day of phantom test failures).
    node_bin = os.path.join(
        os.environ['COLCON_PREFIX_PATH'].split(os.pathsep)[0],
        'map_interface', 'lib', 'map_interface', 'map_server_node')
    proc = subprocess.Popen(
        [node_bin, '--ros-args',
         '-p', 'x_min:=-5.0', '-p', 'x_max:=5.0',
         '-p', 'y_min:=-5.0', '-p', 'y_max:=5.0',
         '-p', 'z_min:=-0.5', '-p', 'z_max:=3.0',
         '-p', 'resolution:=0.2', '-p', 'insert_period_s:=0.0',
         '-p', 'edt_period_s:=0.5'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    feeder = Feeder()
    assert feeder.client.wait_for_service(timeout_sec=15.0), \
        'map_server_node service never came up'

    scene = Scene()
    pts = scene_points(scene)
    for pose in circular_trajectory(radius=1.0, n=15, z=1.0):
        feeder.publish_pose_and_cloud(pose, visible_slice(pts, pose, scene))
        time.sleep(0.1)
    time.sleep(1.5)   # let the EDT timer run

    yield feeder, proc

    feeder.destroy_node()
    rclpy.shutdown()
    proc.terminate()
    proc.wait(timeout=10)


def test_node_survives_full_replay(stack):
    _, proc = stack
    assert proc.poll() is None, 'map_server_node crashed during replay'


def test_pillar_occupied_room_center_free(stack):
    feeder, _ = stack
    res = feeder.query([
        (1.75, 0.05, 1.05),   # pillar 1 near-face surface voxel
        (-1.45, 1.35, 1.05),  # pillar 2 -y-face voxel (faces the trajectory;
                              # the +y face is never visible from the circle)
        (2.0, 0.0, 1.05),     # pillar 1 interior: never observed
        (0.5, -0.5, 1.05),    # open space near trajectory
        (99.0, 0.0, 1.0),     # outside bounds
    ])
    assert res.occupancy[0] == OCC_OCCUPIED
    assert res.occupancy[1] == OCC_OCCUPIED
    # Interior must never be carved free (occlusion working):
    assert res.occupancy[2] != OCC_FREE
    assert res.occupancy[3] == OCC_FREE
    assert res.occupancy[4] == OCC_UNKNOWN
    assert res.clearance[4] == pytest.approx(-1.0)


def test_walls_occupied(stack):
    feeder, _ = stack
    res = feeder.query([(0.0, -3.9, 1.05), (3.9, 0.0, 1.05)])
    assert list(res.occupancy) == [OCC_OCCUPIED, OCC_OCCUPIED]


def test_clearance_gradient(stack):
    feeder, _ = stack
    res = feeder.query([(1.05, 0.05, 1.05),   # ~0.7 m from pillar 1 face
                        (0.05, 0.05, 1.05)])  # room centre-ish, farther
    assert res.clearance[0] > 0.0
    assert res.clearance[1] > res.clearance[0]


def test_query_latency_budget(stack):
    feeder, _ = stack
    probe = [(x * 0.5 - 2.0, y * 0.5 - 2.0, 1.05)
             for x in range(9) for y in range(9)]   # 81 points
    t0 = time.monotonic()
    feeder.query(probe)
    dt_ms = (time.monotonic() - t0) * 1e3
    # Advisory-query budget: a planner goal-check batch must return fast.
    assert dt_ms < 200.0, f'81-point query took {dt_ms:.1f} ms'
