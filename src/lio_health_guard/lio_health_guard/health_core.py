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

        self.stamps.append(t)
        while self.stamps and self.stamps[0] < t - self.window_s:
            self.stamps.popleft()

        clean = True
        if len(self.stamps) / self.window_s < self.min_rate:
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
