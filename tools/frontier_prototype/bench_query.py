# filepath: tools/frontier_prototype/bench_query.py
"""B-01: /map_interface/query latency for explorer-sized band slices.

Runs the INSTALLED map_server_node (wp-c build in ~/colcon_ws/install) against
wp-c's synthetic room scene. Run with localhost-only DDS:
  env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 python3 bench_query.py
"""
import math
import os
import statistics
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from map_interface.srv import QueryMap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from synthetic_scene import Scene, scene_points  # noqa: E402

BIN = os.path.join(HERE, '..', '..', 'install', 'map_interface', 'lib', 'map_interface', 'map_server_node')
RES = 0.2
Z_L = 1.3


def band_points(half_extent):
    eps = 1e-6
    kmin = math.ceil(-half_extent / RES - 0.5 - eps)
    kmax = math.floor(half_extent / RES - 0.5 + eps)
    return [Point(x=(i + 0.5) * RES, y=(j + 0.5) * RES, z=Z_L)
            for i in range(kmin, kmax + 1) for j in range(kmin, kmax + 1)]


def make_cloud(pts):
    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                  for n, o in (('x', 0), ('y', 4), ('z', 8))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('fff', *p) for p in pts)
    return msg


class Bench(Node):
    def __init__(self):
        super().__init__('bench_query')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=2)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', q)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered', q)
        self.cli = self.create_client(QueryMap, '/map_interface/query')

    def feed(self, cloud, z=1.3):
        o = Odometry()
        o.header.frame_id = 'odom'
        o.pose.pose.position.z = z
        o.pose.pose.orientation.w = 1.0
        o.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(o)
        time.sleep(0.05)
        cloud.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(cloud)

    def query(self, points, chunk=None, timeout=30.0):
        chunks = [points] if not chunk else [points[i:i + chunk] for i in range(0, len(points), chunk)]
        t0 = time.monotonic()
        futs = []
        for c in chunks:
            req = QueryMap.Request()
            req.points = c
            futs.append(self.cli.call_async(req))
        occ = []
        for f in futs:
            ev = threading.Event()
            f.add_done_callback(lambda _f, e=ev: e.set())
            if not ev.wait(timeout):
                raise TimeoutError('query timed out')
            occ.extend(f.result().occupancy)
        return (time.monotonic() - t0) * 1e3, occ


def stats(xs):
    xs = sorted(xs)
    p95 = xs[min(len(xs) - 1, int(round(0.95 * (len(xs) - 1))))]
    return f'median {statistics.median(xs):7.1f}  p95 {p95:7.1f}  max {xs[-1]:7.1f} ms'


def main():
    params = ['-p', 'resolution:=0.2', '-p', 'x_min:=-21.0', '-p', 'x_max:=21.0',
              '-p', 'y_min:=-21.0', '-p', 'y_max:=21.0', '-p', 'z_min:=-0.5', '-p', 'z_max:=3.0',
              '-p', 'clearance_cap:=4.0', '-p', 'insert_period_s:=0.5',
              '-p', 'max_insert_range:=8.0', '-p', 'edt_period_s:=2.0',
              '-p', 'map_publish_period_s:=2.0']
    srv = subprocess.Popen([BIN, '--ros-args'] + params,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    rclpy.init()
    node = Bench()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        if not node.cli.wait_for_service(timeout_sec=20.0):
            err = srv.stderr.read1(4000).decode(errors='replace') if srv.poll() is not None else ''
            raise RuntimeError(f'map service not available; server rc={srv.poll()} {err}')
        cloud = make_cloud(scene_points(Scene()))
        print(f'scene points: {cloud.width}')
        for _ in range(6):
            node.feed(cloud)
            time.sleep(0.6)
        time.sleep(2.5)

        sizes = {'2116 (±4.5 m default box)': 4.5, '10000 (±10 m)': 9.99, '40000 (±20 m)': 19.99}
        for label, he in sizes.items():
            pts = band_points(he)
            lat = []
            for _ in range(15):
                ms, occ = node.query(pts)
                lat.append(ms)
            n_free, n_occ, n_unk = occ.count(0), occ.count(1), occ.count(2)
            print(f'warm  N={len(pts):6d} {label:28s} {stats(lat)}   free/occ/unk {n_free}/{n_occ}/{n_unk}')
        pts = band_points(9.99)
        lat = [node.query(pts, chunk=4096)[0] for _ in range(15)]
        print(f'warm  N={len(pts):6d} chunked 4096 x3            {stats(lat)}')

        cold = []
        for _ in range(6):
            node.feed(cloud)
            time.sleep(0.15)           # inserted, EDT dirty, before the 2 s EDT timer
            cold.append(node.query(pts)[0])
            time.sleep(0.6)
        print(f'dirty N={len(pts):6d} post-insertion              {stats(cold)}')
        print(f'server alive: {srv.poll() is None}')
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        srv.terminate()
        try:
            srv.wait(timeout=5)
        except subprocess.TimeoutExpired:
            srv.kill()


if __name__ == '__main__':
    main()
