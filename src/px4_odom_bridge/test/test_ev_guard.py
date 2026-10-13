# filepath: src/px4_odom_bridge/test/test_ev_guard.py
"""Unit tests for the external-vision sanity guard: one or more per rule in
px4_odom_bridge/ev_guard.py, including the boundaries."""
import math

import pytest

from px4_odom_bridge.ev_guard import EvGuard, GuardConfig, State

UNIT = (1.0, 0.0, 0.0, 0.0)
CFG = GuardConfig(max_step_m=0.30, max_gap_s=1.0, max_radius_m=20.0,
                  quat_tolerance=0.01, warn_step_m=0.10)


def guard():
    return EvGuard(CFG)


def accept(g, t, pos=(0.0, 0.0, 0.0), q=UNIT):
    return g.check(t, pos, q)


class TestFiniteness:
    @pytest.mark.parametrize('pos', [(float('nan'), 0, 0), (0, float('inf'), 0),
                                     (0, 0, float('-inf'))])
    def test_non_finite_position_blocks(self, pos):
        g = guard()
        d = accept(g, 0.0, pos)
        assert not d.accept and d.state is State.BLOCKED and 'non-finite' in d.reason

    def test_non_finite_quaternion_blocks(self):
        g = guard()
        assert accept(g, 0.0, q=(float('nan'), 0.0, 0.0, 0.0)).state is State.BLOCKED

    def test_non_finite_time_blocks(self):
        g = guard()
        assert accept(g, float('nan')).state is State.BLOCKED


class TestOrientation:
    def test_unit_quaternion_passes(self):
        s = math.sqrt(0.5)
        assert accept(guard(), 0.0, q=(s, 0.0, 0.0, s)).accept

    def test_within_tolerance_passes(self):
        assert accept(guard(), 0.0, q=(1.005, 0.0, 0.0, 0.0)).accept

    def test_zero_quaternion_blocks(self):
        d = accept(guard(), 0.0, q=(0.0, 0.0, 0.0, 0.0))
        assert not d.accept and 'unit quaternion' in d.reason

    def test_unnormalised_quaternion_blocks(self):
        assert not accept(guard(), 0.0, q=(1.2, 0.0, 0.0, 0.0)).accept


class TestRadius:
    def test_at_limit_passes(self):
        assert accept(guard(), 0.0, (CFG.max_radius_m, 0.0, 0.0)).accept

    def test_above_limit_blocks(self):
        d = accept(guard(), 0.0, (CFG.max_radius_m + 0.001, 0.0, 0.0))
        assert not d.accept and 'max_radius_m' in d.reason

    def test_applies_to_the_first_sample(self):
        """The diverged log starts at 1e4 m: the very first sample must be refused."""
        assert accept(guard(), 0.0, (22687.0, 137683.0, 994128.0)).state is State.BLOCKED


class TestStep:
    def test_at_limit_passes(self):
        g = guard()
        accept(g, 0.0)
        assert accept(g, 0.1, (0.30, 0.0, 0.0)).accept

    def test_above_limit_blocks(self):
        g = guard()
        accept(g, 0.0)
        d = accept(g, 0.1, (0.301, 0.0, 0.0))
        assert not d.accept and 'max_step_m' in d.reason

    def test_the_real_outage_jump_blocks(self):
        g = guard()
        accept(g, 0.0)
        assert not accept(g, 0.1, (3.26, 0.0, 0.0)).accept

    def test_warn_threshold_counts_without_rejecting(self):
        g = guard()
        accept(g, 0.0)
        d = accept(g, 0.1, (0.15, 0.0, 0.0))
        assert d.accept and d.warning and g.counts.warned == 1

    def test_small_steps_do_not_warn(self):
        g = guard()
        accept(g, 0.0)
        d = accept(g, 0.1, (0.05, 0.0, 0.0))
        assert d.accept and not d.warning and g.counts.warned == 0


class TestGap:
    def test_at_limit_passes(self):
        g = guard()
        accept(g, 0.0)
        assert accept(g, 1.0).accept

    def test_above_limit_blocks(self):
        g = guard()
        accept(g, 0.0)
        d = accept(g, 1.001)
        assert not d.accept and 'max_gap_s' in d.reason

    def test_the_real_outage_gap_blocks(self):
        g = guard()
        accept(g, 0.0)
        assert not accept(g, 12.15).accept

    def test_backwards_stamp_blocks(self):
        g = guard()
        accept(g, 10.0)
        d = accept(g, 9.5)
        assert not d.accept and 'backwards' in d.reason

    def test_no_gap_test_on_the_first_sample(self):
        assert accept(guard(), 987654.0).accept


class TestLatching:
    def test_blocked_is_absorbing(self):
        g = guard()
        accept(g, 0.0)
        accept(g, 0.1, (CFG.max_radius_m + 1.0, 0.0, 0.0))            # radius violation
        for i in range(300):                        # 30 s of perfect data
            assert not accept(g, 0.2 + i * 0.1).accept
        assert g.state is State.BLOCKED

    def test_reason_is_kept_after_blocking(self):
        g = guard()
        accept(g, 0.0)
        accept(g, 0.1, (CFG.max_radius_m + 1.0, 0.0, 0.0))
        assert 'max_radius_m' in g.check(0.2, (0, 0, 0), UNIT).reason

    def test_counts_track_both_paths(self):
        g = guard()
        accept(g, 0.0)
        accept(g, 0.1)
        accept(g, 0.2, (CFG.max_radius_m + 1.0, 0.0, 0.0))
        accept(g, 0.3)
        assert g.counts.forwarded == 2 and g.counts.rejected == 2


class TestConfig:
    @pytest.mark.parametrize('kwargs', [
        {'max_step_m': 0.0}, {'max_step_m': -1.0}, {'max_gap_s': float('nan')},
        {'max_radius_m': float('inf')}, {'quat_tolerance': 0.0}, {'warn_step_m': -0.1},
    ])
    def test_invalid_thresholds_rejected(self, kwargs):
        with pytest.raises(ValueError):
            EvGuard(GuardConfig(**kwargs))

    def test_warn_above_reject_is_rejected(self):
        with pytest.raises(ValueError):
            EvGuard(GuardConfig(max_step_m=0.1, warn_step_m=0.2))

    def test_defaults_match_the_spec(self):
        c = GuardConfig()
        assert (c.max_step_m, c.max_gap_s, c.max_radius_m) == (0.30, 1.0, 20.0)


class TestDeterminism:
    def test_identical_sequences_give_identical_decisions(self):
        seq = [(i * 0.1, (i * 0.01, 0.0, 0.0), UNIT) for i in range(50)]
        seq.append((5.0, (99.0, 0.0, 0.0), UNIT))
        a = [EvGuard(CFG).check(*s) for s in seq]
        b = [EvGuard(CFG).check(*s) for s in seq]
        assert a == b

    def test_status_is_readable(self):
        g = guard()
        accept(g, 0.0)
        assert g.status().startswith('FORWARDING forwarded=1')
