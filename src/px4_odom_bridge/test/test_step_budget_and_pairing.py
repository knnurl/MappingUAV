# filepath: src/px4_odom_bridge/test/test_step_budget_and_pairing.py
"""EV guard spec §9.2 (step budget grows with the gap) and the health pairing
for the FAST-LIO fork. Pure logic."""
import pytest

from px4_odom_bridge.ev_guard import EvGuard, GuardConfig, HealthPairer, State

Q = (1.0, 0.0, 0.0, 0.0)


def moving(guard, speed, period, seconds, t0=0.0, x0=0.0):
    t, x, last = t0, x0, None
    for _ in range(int(seconds / period)):
        last = guard.check(t, (x, 0.0, 1.0), Q)
        if not last.accept:
            return last, t, x
        t += period
        x += speed * period
    return last, t, x


def test_budget_equals_max_step_at_the_nominal_rate():
    g = EvGuard()
    assert g.step_budget(0.10) == pytest.approx(0.30)
    assert g.step_budget(0.05) == pytest.approx(0.30)
    assert g.step_budget(0.50) == pytest.approx(0.90)
    assert g.step_budget(1.00) == pytest.approx(1.65)


@pytest.mark.parametrize('hz', [10.0, 5.0, 2.0, 1.2])
def test_flight_at_the_speed_limit_never_latches_whatever_the_rate(hz):
    g = EvGuard()
    d, _, _ = moving(g, speed=1.0, period=1.0 / hz, seconds=20.0)
    assert d.accept and g.state is State.FORWARDING


def test_the_old_fixed_rule_would_have_latched_at_2_hz():
    old = EvGuard(GuardConfig(max_speed_m_s=1e-9))       # budget ~ max_step_m always
    d, _, _ = moving(old, speed=1.0, period=0.5, seconds=5.0)
    assert not d.accept and old.state is State.BLOCKED


def test_a_jump_at_the_nominal_rate_still_blocks():
    g = EvGuard()
    _, t, x = moving(g, 0.5, 0.1, 2.0)
    assert not g.check(t, (x + 0.35, 0.0, 1.0), Q).accept


def test_a_jump_after_a_half_second_gap_still_blocks_beyond_the_budget():
    g = EvGuard()
    _, t, x = moving(g, 0.0, 0.1, 2.0)
    assert g.check(t + 0.4, (x + 0.85, 0.0, 1.0), Q).accept is True    # inside 0.90
    g = EvGuard()
    _, t, x = moving(g, 0.0, 0.1, 2.0)
    assert not g.check(t + 0.4, (x + 1.0, 0.0, 1.0), Q).accept         # beyond 0.90


def test_incident_shape_is_still_caught_by_the_gap_rule():
    g = EvGuard()
    _, t, x = moving(g, 0.0, 0.1, 2.0)
    d = g.check(t + 12.15, (x + 3.26, 0.0, 1.0), Q)
    assert not d.accept and 'gap' in d.reason


def test_bad_speed_config_refused():
    with pytest.raises(ValueError):
        EvGuard(GuardConfig(max_speed_m_s=0.0))


class TestHealthPairer:
    def test_valid_health_releases_the_sample(self):
        p = HealthPairer()
        assert p.offer(1.0, 'a') == []
        assert p.verdict(1.0, True) == [('a', True, '')]
        assert p.verdict(1.0, True) == []                 # nothing held any more

    def test_invalid_health_withholds_it(self):
        p = HealthPairer()
        p.offer(1.0, 'a')
        (sample, forward, why), = p.verdict(1.0, False)
        assert sample == 'a' and not forward and 'no matched points' in why
        assert p.invalid == 1

    def test_lost_health_message_does_not_cut_ev(self):
        p = HealthPairer()
        p.offer(1.0, 'a')
        assert p.offer(1.1, 'b') == [('a', True, 'no health message for this sample')]
        assert p.unpaired == 1
        assert p.verdict(1.1, True) == [('b', True, '')]

    def test_health_for_another_stamp_is_ignored(self):
        p = HealthPairer()
        p.offer(1.0, 'a')
        assert p.verdict(0.9, False) == []
        assert p.verdict(1.0, True) == [('a', True, '')]

    def test_invalid_run_then_recovery_meets_the_guard(self):
        """0.5 s of invalid scans, then valid: the gap is inside max_gap_s and the
        step budget covers real motion, so the guard keeps forwarding."""
        g, p = EvGuard(), HealthPairer()
        t, x = 0.0, 0.0
        for k in range(40):
            valid = not (20 <= k < 25)
            p.offer(t, (t, x))
            for (ts, xs), forward, _ in p.verdict(t, valid):
                if forward:
                    assert g.check(ts, (xs, 0.0, 1.0), Q).accept
            t += 0.1
            x += 0.05
        assert g.state is State.FORWARDING and p.invalid == 5

    def test_long_invalid_run_ends_in_a_block(self):
        g, p = EvGuard(), HealthPairer()
        t = 0.0
        last = None
        for k in range(60):
            valid = not (20 <= k < 35)                    # 1.5 s invalid
            p.offer(t, t)
            for ts, forward, _ in p.verdict(t, valid):
                if forward:
                    last = g.check(ts, (0.0, 0.0, 1.0), Q)
            t += 0.1
        assert g.state is State.BLOCKED and 'gap' in last.reason
