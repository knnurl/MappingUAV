# filepath: src/lio_health_guard/test/test_health_core.py
"""WP-D unit tests: derived LIO health logic."""
import pytest

from lio_health_guard.health_core import HealthMonitor, LioHealth


def feed_steady(mon, t0, seconds, hz=10.0, pos=(0.0, 0.0, 1.0)):
    t = t0
    n = int(seconds * hz)
    for i in range(n):
        t = t0 + i / hz
        mon.update(t, *pos)
    return t


class TestRate:
    def test_steady_10hz_is_ok(self):
        mon = HealthMonitor(min_rate_hz=5.0)
        t = feed_steady(mon, 0.0, 3.0)
        assert mon.state == LioHealth.OK

    def test_slow_stream_degrades(self):
        mon = HealthMonitor(min_rate_hz=5.0)
        feed_steady(mon, 0.0, 2.0, hz=10.0)
        # drop to 2 Hz
        for i in range(6):
            mon.update(2.0 + i * 0.5, 0.0, 0.0, 1.0)
        assert mon.state == LioHealth.DEGRADED
        assert 'rate' in mon.reason

    def test_tick_detects_decay_without_messages(self):
        mon = HealthMonitor(min_rate_hz=5.0)
        feed_steady(mon, 0.0, 2.0)
        assert mon.tick(5.0) == LioHealth.DEGRADED


class TestJump:
    def test_position_jump_degrades(self):
        mon = HealthMonitor(max_speed_m_s=3.0)
        feed_steady(mon, 0.0, 1.0)
        mon.update(1.0, 5.0, 0.0, 1.0)   # 5 m in 0.1 s
        assert mon.state == LioHealth.DEGRADED
        assert 'jump' in mon.reason

    def test_normal_motion_not_flagged(self):
        mon = HealthMonitor(max_speed_m_s=3.0)
        for i in range(30):
            mon.update(i * 0.1, i * 0.1 * 1.0, 0.0, 1.0)   # 1 m/s
        assert mon.state == LioHealth.OK


class TestValues:
    def test_nan_degrades(self):
        mon = HealthMonitor()
        feed_steady(mon, 0.0, 1.0)
        mon.update(1.05, float('nan'), 0.0, 1.0)
        assert mon.state == LioHealth.DEGRADED
        assert 'non-finite' in mon.reason

    def test_inf_degrades(self):
        mon = HealthMonitor()
        mon.update(0.0, float('inf'), 0.0, 1.0)
        assert mon.state == LioHealth.DEGRADED


class TestRecovery:
    def test_recovers_after_clean_window(self):
        mon = HealthMonitor(min_rate_hz=5.0, recover_after_s=2.0)
        feed_steady(mon, 0.0, 1.0)
        mon.update(1.0, 9.0, 0.0, 1.0)          # jump -> DEGRADED
        assert mon.state == LioHealth.DEGRADED
        feed_steady(mon, 1.1, 3.0, pos=(9.0, 0.0, 1.0))
        assert mon.state == LioHealth.OK

    def test_no_early_recovery(self):
        mon = HealthMonitor(min_rate_hz=5.0, recover_after_s=2.0)
        feed_steady(mon, 0.0, 1.0)
        mon.update(1.0, 9.0, 0.0, 1.0)
        feed_steady(mon, 1.1, 1.0, pos=(9.0, 0.0, 1.0))   # only 1 s clean
        assert mon.state == LioHealth.DEGRADED


class TestValidation:
    def test_bad_thresholds_rejected(self):
        with pytest.raises(ValueError):
            HealthMonitor(min_rate_hz=0)
