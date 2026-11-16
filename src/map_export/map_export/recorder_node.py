# filepath: src/map_export/map_export/recorder_node.py
"""map_export recorder (WP-C): accumulates /cloud_registered to disk in
bounded write-through chunks during MAPPING missions. Offline-postprocess
produces the deliverable map; this node only records.

Policy [FIXED, masterplan §5]: offline only relative to autonomy — never run
during flight autonomy. Refuses to start below the same 4 GB free-disk bar
as the flight logger.
"""
import os
import shutil
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from .pcd_io import write_pcd

MIN_FREE_BYTES = 4 * 1024**3


class MapExportRecorder(Node):
    def __init__(self):
        super().__init__('map_export_recorder')
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/map_exports'))
        self.declare_parameter('chunk_points', 500_000)
        self.declare_parameter('max_chunks', 120)   # hard disk bound

        root = self.get_parameter('output_dir').value
        self.session_dir = os.path.join(
            root, time.strftime('session_%Y%m%d_%H%M%S'))
        if shutil.disk_usage(root if os.path.isdir(root) else '/home').free \
                < MIN_FREE_BYTES:
            raise RuntimeError('below 4 GB free disk; refusing to record')
        os.makedirs(self.session_dir, exist_ok=True)

        self.chunk_points = int(self.get_parameter('chunk_points').value)
        self.max_chunks = int(self.get_parameter('max_chunks').value)
        self.buf = []
        self.chunk_idx = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 self.on_cloud, qos)
        self.get_logger().info(f'recording /cloud_registered -> {self.session_dir}')

    def on_cloud(self, msg: PointCloud2):
        if self.chunk_idx >= self.max_chunks:
            self.get_logger().error('max_chunks reached; dropping clouds '
                                    '(bound is deliberate, raise param if needed)')
            return
        step = msg.point_step
        # x,y,z assumed at offsets 0/4/8 float32 (FAST-LIO layout); verified
        # against the driver in the replay harness.
        data = msg.data
        for i in range(msg.width * msg.height):
            o = i * step
            self.buf.append(struct.unpack_from('<fff', data, o))
        if len(self.buf) >= self.chunk_points:
            self.flush()

    def flush(self):
        if not self.buf:
            return
        if shutil.disk_usage(self.session_dir).free < MIN_FREE_BYTES:
            self.get_logger().error('below 4 GB free; stopping chunk writes')
            self.buf.clear()
            self.chunk_idx = self.max_chunks
            return
        path = os.path.join(self.session_dir, f'chunk_{self.chunk_idx:04d}.pcd')
        n = write_pcd(path, self.buf)
        self.get_logger().info(f'wrote {n} pts -> {path}')
        self.buf.clear()
        self.chunk_idx += 1


def main():
    rclpy.init()
    node = MapExportRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
