# filepath: src/planner_shim/test/test_shim_core.py
"""WP-D unit tests: go_to_with_avoidance validity checks."""
import pytest

from planner_shim.shim_core import EnuBox, Limits, check_goal, check_trajectory

BOX = EnuBox(-5.0, 5.0, -5.0, 5.0, 0.0, 2.5)
LIM = Limits(max_vel=1.0, max_acc=2.0)


def straight_traj(speed=0.5, duration=4.0, dt=0.1, z=1.0):
    return [(i * dt, i * dt * speed, 0.0, z)
            for i in range(int(duration / dt) + 1)]


class TestGoal:
    def test_center_goal_ok(self):
        ok, _ = check_goal((0.0, 0.0, 1.0), BOX, inflation=0.4)
        assert ok

    def test_goal_outside_rejected(self):
        ok, why = check_goal((9.0, 0.0, 1.0), BOX, inflation=0.4)
        assert not ok and 'outside' in why

    def test_goal_inside_but_within_inflation_rejected(self):
        ok, why = check_goal((4.8, 0.0, 1.0), BOX, inflation=0.4)
        assert not ok

    def test_nan_goal_rejected(self):
        ok, why = check_goal((float('nan'), 0.0, 1.0), BOX, inflation=0.4)
        assert not ok and 'non-finite' in why


class TestTrajectory:
    def test_gentle_straight_line_ok(self):
        ok, why = check_trajectory(straight_traj(speed=0.5), BOX, LIM)
        assert ok, why

    def test_overspeed_rejected(self):
        ok, why = check_trajectory(straight_traj(speed=1.5), BOX, LIM)
        assert not ok and 'velocity' in why

    def test_escaping_box_rejected(self):
        ok, why = check_trajectory(straight_traj(speed=0.9, duration=8.0),
                                   BOX, LIM)
        assert not ok and 'planning box' in why

    def test_non_monotonic_time_rejected(self):
        traj = straight_traj()
        traj[5] = (traj[4][0], *traj[5][1:])
        ok, why = check_trajectory(traj, BOX, LIM)
        assert not ok and 'monotonic' in why

    def test_too_short_rejected(self):
        ok, why = check_trajectory([(0.0, 0.0, 0.0, 1.0)], BOX, LIM)
        assert not ok

    def test_nan_sample_rejected(self):
        traj = straight_traj()
        traj[3] = (traj[3][0], float('inf'), 0.0, 1.0)
        ok, why = check_trajectory(traj, BOX, LIM)
        assert not ok and 'non-finite' in why

    def test_hard_brake_rejected(self):
        # 1.0 m/s to 0 within one 0.1 s step = 10 m/s^2 > 2.0 limit
        traj = [(0.0, 0.0, 0.0, 1.0), (0.1, 0.1, 0.0, 1.0),
                (0.2, 0.2, 0.0, 1.0), (0.3, 0.2, 0.0, 1.0),
                (0.4, 0.2, 0.0, 1.0)]
        ok, why = check_trajectory(traj, BOX, LIM)
        assert not ok and 'acceleration' in why


class TestValidation:
    def test_invalid_box_rejected(self):
        with pytest.raises(ValueError):
            EnuBox(5.0, -5.0, -5.0, 5.0, 0.0, 2.5)

    def test_invalid_limits_rejected(self):
        with pytest.raises(ValueError):
            Limits(max_vel=0.0, max_acc=2.0)
