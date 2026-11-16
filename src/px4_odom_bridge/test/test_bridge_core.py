# filepath: src/px4_odom_bridge/test/test_bridge_core.py
"""Unit tests for the bridge's pure helpers: frame convention, stamp gating,
sole-writer detection. The frame tests pin the convention that
scripts/verify_gate3.sh test A checks by hand."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from px4_odom_bridge.odom_bridge_node import (
    enu_flu_to_ned_frd, other_writers, sample_age_ok, stamp_to_us)

IDENTITY = [1.0, 0.0, 0.0, 0.0]


def ned_heading(q):
    """Yaw of an FRD body in NED, from q = (w,x,y,z)."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def q_enu_yaw(yaw):
    """FLU body rotated by yaw about ENU up (counter-clockwise seen from above)."""
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


class TestFrameConvention:
    def test_startup_forward_is_east(self):
        pos, _ = enu_flu_to_ned_frd([1.0, 0.0, 0.0], IDENTITY)
        assert pos == pytest.approx([0.0, 1.0, 0.0])

    def test_startup_right_is_south(self):
        pos, _ = enu_flu_to_ned_frd([0.0, -1.0, 0.0], IDENTITY)
        assert pos == pytest.approx([-1.0, 0.0, 0.0])

    def test_up_is_negative_down(self):
        pos, _ = enu_flu_to_ned_frd([0.0, 0.0, 0.5], IDENTITY)
        assert pos == pytest.approx([0.0, 0.0, -0.5])

    def test_startup_heading_is_east(self):
        _, q = enu_flu_to_ned_frd([0.0, 0.0, 0.0], IDENTITY)
        assert ned_heading(q) == pytest.approx(math.pi / 2)

    def test_clockwise_yaw_increases_heading(self):
        _, q = enu_flu_to_ned_frd([0.0, 0.0, 0.0], q_enu_yaw(-math.pi / 2))
        assert math.cos(ned_heading(q)) == pytest.approx(-1.0)  # heading pi

    @pytest.mark.parametrize('yaw', [0.0, 0.3, 1.2, -2.0, 3.0])
    def test_heading_agrees_with_position(self, yaw):
        # Step 1 m along the body's forward axis: the NED displacement must
        # point along the heading PX4 reads from q.
        forward_enu = [math.cos(yaw), math.sin(yaw), 0.0]
        pos, q = enu_flu_to_ned_frd(forward_enu, q_enu_yaw(yaw))
        psi = ned_heading(q)
        assert pos[:2] == pytest.approx([math.cos(psi), math.sin(psi)], abs=1e-9)

    def test_output_quaternion_is_unit(self):
        _, q = enu_flu_to_ned_frd([0.0, 0.0, 0.0], [2.0, 0.1, -0.3, 0.7])
        assert np.linalg.norm(q) == pytest.approx(1.0)


class TestStampGate:
    MAX = 500_000

    def test_stamp_to_us(self):
        assert stamp_to_us(SimpleNamespace(sec=1, nanosec=500_000_999)) == 1_500_000

    def test_fresh_sample_ok(self):
        assert sample_age_ok(10_000_000, 10_030_000, self.MAX)

    def test_zero_age_ok(self):
        assert sample_age_ok(10_000_000, 10_000_000, self.MAX)

    def test_age_at_limit_ok(self):
        assert sample_age_ok(10_000_000, 10_500_000, self.MAX)

    def test_stale_sample_dropped(self):
        assert not sample_age_ok(10_000_000, 10_500_001, self.MAX)

    def test_future_sample_dropped(self):
        assert not sample_age_ok(10_000_001, 10_000_000, self.MAX)

    def test_zero_stamp_dropped(self):
        assert not sample_age_ok(0, 1_757_000_000_000_000, self.MAX)


def _info(ns, name):
    return SimpleNamespace(node_namespace=ns, node_name=name)


class TestOtherWriters:
    def test_only_self(self):
        assert other_writers([_info('/', 'px4_odom_bridge')], '/px4_odom_bridge') == []

    def test_platform_in_namespace_reported(self):
        infos = [_info('/', 'px4_odom_bridge'), _info('/drone0', 'platform')]
        assert other_writers(infos, '/px4_odom_bridge') == ['/drone0/platform']

    def test_no_publishers(self):
        assert other_writers([], '/px4_odom_bridge') == []
