# filepath: src/geofence_watchdog/test/test_node_land_rules.py
"""Node-level checks for the armed gate, the stop-after-Land rule and the
px4_odom_bridge state input. Live rclpy graph in one process, no hardware."""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from px4_msgs.msg import VehicleLocalPosition, VehicleCommand, VehicleStatus
from std_msgs.msg import UInt8

from geofence_watchdog.watchdog_node import GeofenceWatchdog


class Harness:
    def __init__(self, overrides=None):
        params = [Parameter(k, value=v) for k, v in (overrides or {}).items()]
        self.node = GeofenceWatchdog(parameter_overrides=params)
        self.helper = rclpy.create_node('watchdog_land_rules_helper')
        self.pos_pub = self.helper.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        self.status_pub = self.helper.create_publisher(
            VehicleStatus, '/fmu/out/vehicle_status', 10)
        self.bridge_pub = self.helper.create_publisher(
            UInt8, '/px4_odom_bridge/state', 10)
        self.commands = []
        self.helper.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command',
            lambda m: self.commands.append(m), 10)
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        self.exec.add_node(self.helper)

    def pos(self, x, y, z):
        msg = VehicleLocalPosition()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        msg.xy_valid = msg.z_valid = True
        self.pos_pub.publish(msg)

    def status(self, armed, land=False):
        msg = VehicleStatus()
        msg.arming_state = (VehicleStatus.ARMING_STATE_ARMED if armed
                            else VehicleStatus.ARMING_STATE_DISARMED)
        msg.nav_state = (VehicleStatus.NAVIGATION_STATE_AUTO_LAND if land
                         else VehicleStatus.NAVIGATION_STATE_POSCTL)
        self.status_pub.publish(msg)

    def spin_for(self, seconds, tick=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if tick:
                tick()
            self.exec.spin_once(timeout_sec=0.05)

    def lands(self):
        return [c for c in self.commands
                if c.command == VehicleCommand.VEHICLE_CMD_NAV_LAND]

    def shutdown(self):
        self.exec.remove_node(self.node)
        self.exec.remove_node(self.helper)
        self.node.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def make():
    rclpy.init()
    made = []

    def _make(overrides=None):
        h = Harness(overrides)
        made.append(h)
        h.spin_for(0.5)
        return h
    yield _make
    for h in made:
        h.shutdown()
    rclpy.shutdown()


def test_disarmed_vehicle_is_not_commanded(make):
    h = make()
    h.spin_for(1.5, lambda: (h.status(False), h.pos(5.0, 0.0, -1.0)))
    assert h.lands() == []


def test_armed_hard_breach_lands_then_stops_after_px4_confirms(make):
    h = make()
    h.spin_for(1.2, lambda: (h.status(True), h.pos(5.0, 0.0, -1.0)))
    first = len(h.lands())
    assert first >= 1
    # PX4 enters Land; then the pilot takes over (Position) still outside.
    h.spin_for(0.6, lambda: (h.status(True, land=True), h.pos(5.0, 0.0, -1.0)))
    confirmed_at = len(h.lands())
    h.spin_for(2.5, lambda: (h.status(True), h.pos(5.0, 0.0, -1.0)))
    assert len(h.lands()) == confirmed_at


def test_bridge_blocked_lands_inside_the_box(make):
    h = make()

    def tick():
        h.status(True)
        h.pos(0.0, 0.0, -1.0)
        h.bridge_pub.publish(UInt8(data=3))
    h.spin_for(1.0, tick)
    assert len(h.lands()) >= 1


def test_bridge_forwarding_is_quiet(make):
    h = make()

    def tick():
        h.status(True)
        h.pos(0.0, 0.0, -1.0)
        h.bridge_pub.publish(UInt8(data=1))
    h.spin_for(1.0, tick)
    assert h.lands() == []


def test_bridge_state_silence_lands(make):
    h = make({'bridge_timeout_s': 0.5})

    def healthy():
        h.status(True)
        h.pos(0.0, 0.0, -1.0)
        h.bridge_pub.publish(UInt8(data=1))
    h.spin_for(0.6, healthy)
    assert h.lands() == []
    h.spin_for(1.2, lambda: (h.status(True), h.pos(0.0, 0.0, -1.0)))
    assert len(h.lands()) >= 1


def test_bridge_input_can_be_disabled(make):
    h = make({'use_bridge_state': False})

    def tick():
        h.status(True)
        h.pos(0.0, 0.0, -1.0)
        h.bridge_pub.publish(UInt8(data=3))
    h.spin_for(1.0, tick)
    assert h.lands() == []


def test_soft_then_hard_breach_still_lands(make):
    """Regression: the action log changed severity at one call site, rclpy
    raised, and the watchdog died exactly when it had to land."""
    h = make()
    h.spin_for(0.6, lambda: (h.status(True), h.pos(0.0, 0.0, -1.0)))     # OK   (info)
    h.spin_for(0.6, lambda: (h.status(True), h.pos(1.2, 0.0, -1.0)))     # SOFT (warn)
    assert h.lands() == []
    h.spin_for(1.0, lambda: (h.status(True), h.pos(5.0, 0.0, -1.0)))     # HARD (error)
    assert len(h.lands()) >= 1
    h.spin_for(0.6, lambda: (h.status(True), h.pos(0.0, 0.0, -1.0)))     # still latched
