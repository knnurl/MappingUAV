# filepath: src/lio_health_guard/lio_health_guard/health_core.py
"""Pure logic for the LIO degeneracy guard (WP-D, masterplan §6).

[VERIFY outcome 2026-07-05]: the FAST_LIO ROS2 branch publishes NO health or
degeneracy topic (laserMapping.cpp exposes clouds, /Odometry, /path only).
Health is therefore DERIVED from /Odometry itself:
  - rate: too few messages inside a sliding window;
  - jump: position discontinuity implying impossible velocity;
  - values: any non-finite field.
Crude is fine; present is mandatory (masterplan wording). The geofence
watchdog independently lands on full LIO silence > 1 s; this guard covers
the degraded-but-alive band and requests hover.
"""
import math
from collections import deque
from enum import IntEnum


class LioHealth(IntEnum):
    OK = 0
    DEGRADED = 1


class HealthMonitor:
    """Feed odometry samples via update(); read .state.

    Recovery requires recover_after_s of clean samples (hysteresis in time,
    same rationale as the geofence state machine).
    """

    def __init__(self, min_rate_hz=5.0, window_s=1.0,
                 max_speed_m_s=3.0, recover_after_s=2.0):
        if min_rate_hz <= 0 or window_s <= 0 or max_speed_m_s <= 0:
            raise ValueError('thresholds must be positive')
        self.min_rate = min_rate_hz
        self.window_s = window_s
        self.max_speed = max_speed_m_s
        self.recover_after_s = recover_after_s
        self.stamps = deque()
        self.last = None            # (t, x, y, z)
        self.state = LioHealth.OK
        self._degraded_reason = ''
        self._clean_since = None
        self._first_t = None

    @property
    def reason(self):
        return self._degraded_reason

    def _degrade(self, why):
        self.state = LioHealth.DEGRADED
        self._degraded_reason = why
        self._clean_since = None

    def update(self, t, x, y, z):
        """New odometry sample at time t (seconds). Returns current state."""
        if not all(math.isfinite(v) for v in (x, y, z)):
            self._degrade(f'non-finite position at t={t:.3f}')
            self.last = None            # do not jump-check against garbage
            return self.state

        if self.last is not None:
            dt = t - self.last[0]
            if dt > 0:
                dist = math.dist((x, y, z), self.last[1:])
                if dist / dt > self.max_speed:
                    self._degrade(
                        f'position jump {dist:.2f} m in {dt:.3f} s '
                        f'(> {self.max_speed} m/s)')
        self.last = (t, x, y, z)

        if self._first_t is None:
            self._first_t = t
        self.stamps.append(t)
        while self.stamps and self.stamps[0] < t - self.window_s:
            self.stamps.popleft()

        clean = True
        # The window is not full until window_s after the first sample: before
        # that a healthy 10 Hz stream reads as "1 Hz" and the guard used to
        # start DEGRADED for recover_after_s (seen on a replay, 2026-11-16).
        warmed_up = t - self._first_t >= self.window_s
        if warmed_up and len(self.stamps) / self.window_s < self.min_rate:
            self._degrade(
                f'rate {len(self.stamps) / self.window_s:.1f} Hz '
                f'< {self.min_rate} Hz')
            clean = False

        if self.state == LioHealth.DEGRADED and clean \
                and self._degraded_reason.startswith(('rate', 'position', 'non-finite')):
            if self._clean_since is None:
                self._clean_since = t
            elif t - self._clean_since >= self.recover_after_s:
                self.state = LioHealth.OK
                self._degraded_reason = ''
                self._clean_since = None
        return self.state

    def tick(self, t):
        """Call periodically even without messages: rate decay check."""
        while self.stamps and self.stamps[0] < t - self.window_s:
            self.stamps.popleft()
        if len(self.stamps) / self.window_s < self.min_rate:
            self._degrade('rate decay: '
                          f'{len(self.stamps) / self.window_s:.1f} Hz')
        return self.state


class NativeHealthMonitor:
    """Judges the FAST-LIO fork's per-scan health message (interface/LioHealth).

    DEGRADED when, for persist_s without a healthy scan in between: the scan is
    invalid (no point matched; the odometry is IMU prediction), or translation
    degeneracy, match ratio or residual is outside its limit. Also DEGRADED when
    no health message has arrived for silence_s. Recovery needs recover_after_s
    of healthy scans.

    The default limits are PROVISIONAL (fork docs/lio_health.md, 2026-11-16):
    about half the healthiest minimum seen on three recordings, none of which
    degraded. Healthy envelope seen: degeneracy >= 0.18, ratio >= 0.41,
    residual <= 0.066 m.
    """

    def __init__(self, min_translation_degeneracy=0.08, min_match_ratio=0.25,
                 max_residual_rms_m=0.10, persist_s=0.5, silence_s=1.0,
                 recover_after_s=2.0):
        if not (0 <= min_translation_degeneracy <= 1 and 0 <= min_match_ratio <= 1):
            raise ValueError('degeneracy and ratio limits must be in [0, 1]')
        if max_residual_rms_m <= 0 or persist_s < 0 or silence_s <= 0:
            raise ValueError('limits must be positive')
        self.min_deg = min_translation_degeneracy
        self.min_ratio = min_match_ratio
        self.max_rms = max_residual_rms_m
        self.persist_s = persist_s
        self.silence_s = silence_s
        self.recover_after_s = recover_after_s
        self.state = LioHealth.OK
        self.reason = ''
        self.last_t = None
        self._bad_since = None
        self._bad_why = ''
        self._clean_since = None

    def judge(self, valid, translation_degeneracy, match_ratio, residual_rms):
        """'' for a healthy scan, otherwise why it is not."""
        values = (translation_degeneracy, match_ratio, residual_rms)
        if not valid:
            return 'no point matched (IMU prediction only)'
        if not all(math.isfinite(v) for v in values):
            return 'non-finite health values'
        if translation_degeneracy < self.min_deg:
            return f'translation degeneracy {translation_degeneracy:.2f} < {self.min_deg}'
        if match_ratio < self.min_ratio:
            return f'match ratio {match_ratio:.2f} < {self.min_ratio}'
        if residual_rms > self.max_rms:
            return f'residual {residual_rms:.3f} m > {self.max_rms}'
        return ''

    def update(self, t, valid, translation_degeneracy, match_ratio, residual_rms):
        self.last_t = t
        why = self.judge(valid, translation_degeneracy, match_ratio, residual_rms)
        if why:
            self._clean_since = None
            if self._bad_since is None:
                self._bad_since, self._bad_why = t, why
            if t - self._bad_since >= self.persist_s:
                self.state, self.reason = LioHealth.DEGRADED, why
        else:
            self._bad_since = None
            if self.state == LioHealth.DEGRADED:
                if self._clean_since is None:
                    self._clean_since = t
                elif t - self._clean_since >= self.recover_after_s:
                    self.state, self.reason = LioHealth.OK, ''
                    self._clean_since = None
        return self.state

    def tick(self, t, started_t):
        """Silence check. started_t: when the caller began listening."""
        last = self.last_t if self.last_t is not None else started_t
        if t - last > self.silence_s:
            self.state = LioHealth.DEGRADED
            self.reason = f'no LIO health message for {t - last:.1f} s'
            self._clean_since = None
        return self.state


def combine(*monitors):
    """Worst state of several monitors and the reasons of the degraded ones."""
    bad = [m.reason for m in monitors if m.state == LioHealth.DEGRADED]
    return (LioHealth.DEGRADED if bad else LioHealth.OK), '; '.join(bad)
