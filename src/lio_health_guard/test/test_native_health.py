# filepath: src/lio_health_guard/test/test_native_health.py
"""Judging the FAST-LIO fork's health message. Pure logic."""
import pytest

from lio_health_guard.health_core import (
    HealthMonitor, LioHealth, NativeHealthMonitor, combine,
)

GOOD = (True, 0.40, 0.56, 0.042)        # medians from the 2026-10-23 recordings
WORST_HEALTHY = (True, 0.18, 0.41, 0.066)   # the healthiest envelope's edge


def run(mon, t0, seconds, sample, hz=10.0):
    t = t0
    for k in range(int(seconds * hz)):
        t = t0 + k / hz
        mon.update(t, *sample)
    return t + 1.0 / hz


def test_recorded_healthy_envelope_never_degrades():
    m = NativeHealthMonitor()
    t = run(m, 0.0, 30.0, GOOD)
    run(m, t, 30.0, WORST_HEALTHY)
    assert m.state == LioHealth.OK


@pytest.mark.parametrize('sample,word', [
    ((False, 0.40, 0.56, 0.042), 'IMU prediction'),
    ((True, 0.05, 0.56, 0.042), 'degeneracy'),
    ((True, 0.40, 0.20, 0.042), 'match ratio'),
    ((True, 0.40, 0.56, 0.15), 'residual'),
    ((True, float('nan'), 0.56, 0.042), 'non-finite'),
])
def test_each_fault_degrades_after_the_persistence_time(sample, word):
    m = NativeHealthMonitor()
    t = run(m, 0.0, 2.0, GOOD)
    m.update(t, *sample)
    assert m.state == LioHealth.OK                 # one bad scan is not enough
    run(m, t + 0.1, 0.6, sample)
    assert m.state == LioHealth.DEGRADED and word in m.reason


def test_a_healthy_scan_in_between_resets_the_persistence():
    m = NativeHealthMonitor()
    bad = (True, 0.05, 0.56, 0.042)
    t = 0.0
    for _ in range(20):                            # bad, bad, bad, good, repeating
        for s in (bad, bad, bad, GOOD):
            m.update(t, *s)
            t += 0.1
    assert m.state == LioHealth.OK


def test_recovery_needs_sustained_health():
    m = NativeHealthMonitor()
    t = run(m, 0.0, 1.0, (False, 0, 0, 0))
    assert m.state == LioHealth.DEGRADED
    t = run(m, t, 1.5, GOOD)
    assert m.state == LioHealth.DEGRADED
    run(m, t, 1.0, GOOD)
    assert m.state == LioHealth.OK and m.reason == ''


def test_silence_degrades_including_never_received():
    m = NativeHealthMonitor()
    assert m.tick(0.5, started_t=0.0) == LioHealth.OK
    assert m.tick(1.5, started_t=0.0) == LioHealth.DEGRADED
    m = NativeHealthMonitor()
    t = run(m, 0.0, 2.0, GOOD)
    assert m.tick(t + 0.5, 0.0) == LioHealth.OK
    assert m.tick(t + 1.5, 0.0) == LioHealth.DEGRADED and 'no LIO health' in m.reason


def test_bad_limits_rejected():
    with pytest.raises(ValueError):
        NativeHealthMonitor(min_match_ratio=1.5)
    with pytest.raises(ValueError):
        NativeHealthMonitor(max_residual_rms_m=0.0)


def test_combine_takes_the_worst():
    derived, native = HealthMonitor(), NativeHealthMonitor()
    assert combine(derived, native) == (LioHealth.OK, '')
    run(native, 0.0, 1.0, (False, 0, 0, 0))
    state, reason = combine(derived, native)
    assert state == LioHealth.DEGRADED and 'IMU prediction' in reason
