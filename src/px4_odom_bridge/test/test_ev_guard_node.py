# filepath: src/px4_odom_bridge/test/test_ev_guard_node.py
"""Node-level tests: the guard inside the real bridge node.

A fake FAST-LIO publishes /Odometry; the test watches what reaches
/fmu/in/vehicle_visual_odometry and /px4_odom_bridge/status.

SAFETY: skipped unless ROS_DOMAIN_ID is set and not 0 (PX4's default domain), so
these cannot reach a flight controller. Run with the XRCE agent stopped.
"""
import os
import threading
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from px4_odom_bridge.odom_bridge_node import OdomBridge

DOMAIN = os.environ.get('ROS_DOMAIN_ID', '')
pytestmark = pytest.mark.skipif(
    DOMAIN in ('', '0'),
    reason='set ROS_DOMAIN_ID to a non-zero value: domain 0 is the flight controller\'s')


class FakeLio(Node):
    def __init__(self):
        super().__init__('fake_lio')
        self.pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.ev = []
        self.status = []
        self.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', lambda m: self.ev.append(m), 10)
        self.create_subscription(
            String, '/px4_odom_bridge/status', lambda m: self.status.append(m.data), 10)

    def send(self, x=0.0, y=0.0, z=0.0, qw=1.0):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = qw
        self.pub.publish(msg)


@pytest.fixture
def rig():
    if not rclpy.ok():
        rclpy.init()
    bridge = OdomBridge()
    fake = FakeLio()
    ex = MultiThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(fake)
    thread = threading.Thread(target=ex.spin, daemon=True)
    thread.start()
    time.sleep(1.0)                      # discovery
    yield fake, bridge
    ex.shutdown()
    bridge.destroy_node()
    fake.destroy_node()
    thread.join(timeout=2.0)


def stream(fake, n, **kw):
    for _ in range(n):
        fake.send(**kw)
        time.sleep(0.05)


class TestHealthyStream:
    def test_samples_reach_px4(self, rig):
        fake, _ = rig
        stream(fake, 10)
        time.sleep(0.5)
        assert len(fake.ev) >= 8
        assert any(s.startswith('FORWARDING') for s in fake.status)


class TestBlocking:
    def test_nan_stops_the_stream_permanently(self, rig):
        fake, bridge = rig
        stream(fake, 5)
        time.sleep(0.3)
        before = len(fake.ev)
        assert before >= 3
        fake.send(x=float('nan'))
        time.sleep(0.3)
        stream(fake, 20)                 # a further second of perfect data
        time.sleep(0.5)
        assert len(fake.ev) == before, 'EV kept flowing after a non-finite sample'
        assert any(s.startswith('BLOCKED') for s in fake.status)
        assert 'non-finite' in bridge.guard.reason

    def test_jump_blocks(self, rig):
        fake, bridge = rig
        stream(fake, 5)
        time.sleep(0.3)
        before = len(fake.ev)
        fake.send(x=3.26)                # the real outage jump
        time.sleep(0.3)
        stream(fake, 10)
        time.sleep(0.4)
        assert len(fake.ev) == before
        assert 'jumped' in bridge.guard.reason

    def test_radius_blocks(self, rig):
        fake, bridge = rig
        fake.send(x=0.0)
        time.sleep(0.2)
        fake.send(x=0.0, y=0.0, z=0.0)
        time.sleep(0.2)
        before = len(fake.ev)
        # radius is checked before the step rule, so this blocks on radius
        fake.send(x=bridge.guard.config.max_radius_m + 1.0)
        time.sleep(0.3)
        stream(fake, 5)
        time.sleep(0.3)
        assert len(fake.ev) == before
        assert 'max_radius_m' in bridge.guard.reason


class TestStatusTopic:
    def test_status_published_in_every_state(self, rig):
        fake, _ = rig
        stream(fake, 24)                     # keep streaming across a status tick
        assert any(s.startswith('FORWARDING') for s in fake.status)
        fake.send(x=float('inf'))
        time.sleep(1.5)
        assert any(s.startswith('BLOCKED') for s in fake.status)

    def test_silence_reports_stale(self, rig):
        """A silent LIO must not leave the status reading FORWARDING (2026-09-14 bench run)."""
        fake, _ = rig
        stream(fake, 24)
        before = len(fake.ev)
        time.sleep(2.5)                      # > max_gap_s plus a 1 Hz status period
        assert fake.status[-1].startswith('STALE'), fake.status[-3:]
        assert len(fake.ev) == before
