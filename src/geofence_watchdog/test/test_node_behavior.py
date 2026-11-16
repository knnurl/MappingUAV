# filepath: src/geofence_watchdog/test/test_node_behavior.py
"""WP-E node-level behavior tests, executed against a live rclpy graph in one
process (no hardware, no launch files). Verifies the wiring the pure-logic
tests cannot: subscriptions, the action ladder driving real publications, and
NAV_LAND emission on /fmu/in/vehicle_command.
"""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from px4_msgs.msg import VehicleLocalPosition, VehicleCommand
from std_msgs.msg import Empty

from geofence_watchdog.watchdog_node import GeofenceWatchdog


class Harness:
    def __init__(self):
        self.node = GeofenceWatchdog()
        self.helper = rclpy.create_node('watchdog_test_helper')
        self.pos_pub = self.helper.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        self.commands = []
        self.hovers = []
        self.helper.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command',
            lambda m: self.commands.append(m), 10)
        self.helper.create_subscription(
            Empty, '/geofence/hover_request',
            lambda m: self.hovers.append(m), 10)
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        self.exec.add_node(self.helper)

    def publish_pos(self, x, y, z, valid=True):
        msg = VehicleLocalPosition()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        msg.xy_valid = valid
        msg.z_valid = valid
        self.pos_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.exec.spin_once(timeout_sec=0.05)

    def shutdown(self):
        self.exec.remove_node(self.node)
        self.exec.remove_node(self.helper)
        self.node.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def harness():
    rclpy.init()
    h = Harness()
    # Let discovery settle so publications are not dropped pre-match.
    h.spin_for(0.5)
    yield h
    h.shutdown()
    rclpy.shutdown()


def test_inside_box_no_action(harness):
    for _ in range(5):
        harness.publish_pos(0.0, 0.0, -1.0)
        harness.spin_for(0.1)
    assert harness.commands == []
    assert harness.hovers == []


def test_soft_breach_requests_hover_not_land(harness):
    # Default box x_max=1.5, margin 0.5: x=1.2 is in the band.
    for _ in range(5):
        harness.publish_pos(1.2, 0.0, -1.0)
        harness.spin_for(0.1)
    assert len(harness.hovers) >= 1
    assert harness.commands == []


def test_hard_breach_sends_nav_land(harness):
    harness.publish_pos(5.0, 0.0, -1.0)   # far outside
    harness.spin_for(0.5)
    assert len(harness.commands) >= 1
    cmd = harness.commands[0]
    assert cmd.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert cmd.from_external is True

def test_land_resends_but_rate_limited(harness):
    harness.publish_pos(5.0, 0.0, -1.0)
    harness.spin_for(2.3)                  # resend period 1.0 s
    n = len(harness.commands)
    assert 2 <= n <= 4                     # ~1 initial + ~2 resends


def test_estimator_invalid_lands_even_inside_box(harness):
    harness.publish_pos(0.0, 0.0, -1.0, valid=False)
    harness.spin_for(0.5)
    assert len(harness.commands) >= 1
    assert harness.commands[0].command == VehicleCommand.VEHICLE_CMD_NAV_LAND
