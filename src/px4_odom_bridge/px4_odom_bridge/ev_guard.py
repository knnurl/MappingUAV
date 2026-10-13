# filepath: src/px4_odom_bridge/px4_odom_bridge/ev_guard.py
"""Sanity guard for the external-vision stream sent to EKF2.

Why this exists (docs/px4_odom_bridge_ev_guard_spec.md, gate3 transcript section 10):
on 2026-09-14 the Mid-360 lost power, FAST-LIO's state ran away, and the bridge
forwarded positions of hundreds of thousands of metres. EKF2 rejected them and
then reset its position TO the vision measurement 123 times, destroying its own
estimate. EKF2 cannot protect itself from a confidently wrong vision source, so
the containment has to sit here, on the data path.

Rules, in evaluation order. Every one of them latches except the caller's own
stamp-age check, which stays a plain drop because clock skew is transient:
  G1 every value finite
  G2 orientation is a unit quaternion
  G3 |position| <= max_radius_m            gradual-runaway backstop
  G4 stamps do not go backwards
  G5 gap since the last accepted sample <= max_gap_s
  G6 step since the last accepted sample <= max_step_m

Once blocked, the guard stays blocked for the life of the process: FAST-LIO does
not recover on its own, and a restarted FAST-LIO has a NEW origin, so resuming
would hand EKF2 a step change - the same failure by another route. A blocked
bridge means EV stops, which PX4 and the geofence watchdog already handle.

Thresholds are defended in the spec against 470 s of measured healthy traffic
(step median 3.67 mm, p99.9 104 mm; interval median 100 ms, p99.9 131 ms).
"""
import math
from dataclasses import dataclass
from enum import Enum


class State(Enum):
    """WAITING -> FORWARDING -> BLOCKED. BLOCKED is absorbing."""

    WAITING = 'WAITING'
    FORWARDING = 'FORWARDING'
    BLOCKED = 'BLOCKED'


@dataclass(frozen=True)
class GuardConfig:
    max_step_m: float = 0.30
    max_gap_s: float = 1.0
    max_radius_m: float = 20.0
    quat_tolerance: float = 0.01
    warn_step_m: float = 0.10

    def validate(self) -> None:
        """Raise ValueError on any unusable threshold; the node refuses to start."""
        for name in ('max_step_m', 'max_gap_s', 'max_radius_m', 'quat_tolerance',
                     'warn_step_m'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite number > 0, got {value!r}')
        if self.warn_step_m > self.max_step_m:
            raise ValueError(
                f'warn_step_m ({self.warn_step_m}) must not exceed max_step_m ({self.max_step_m})')


@dataclass(frozen=True)
class Decision:
    accept: bool
    state: State
    reason: str = ''
    warning: str = ''


@dataclass
class Counts:
    forwarded: int = 0
    rejected: int = 0
    warned: int = 0


class EvGuard:
    """Decides, per sample, whether external vision may reach EKF2."""

    def __init__(self, config: GuardConfig = None):
        self.config = config if config is not None else GuardConfig()
        self.config.validate()
        self.state = State.WAITING
        self.reason = ''
        self.counts = Counts()
        self._last_t = None
        self._last_position = None

    def check(self, t_s: float, position, quaternion) -> Decision:
        """position: (x, y, z) in any consistent frame. quaternion: (w, x, y, z)."""
        if self.state is State.BLOCKED:
            self.counts.rejected += 1
            return Decision(False, self.state, self.reason)

        position = tuple(float(v) for v in position)
        quaternion = tuple(float(v) for v in quaternion)

        if not all(math.isfinite(v) for v in (t_s,) + position + quaternion):
            return self._block('sample contains a non-finite value')

        norm = math.sqrt(sum(v * v for v in quaternion))
        if abs(norm - 1.0) > self.config.quat_tolerance:
            return self._block(f'orientation is not a unit quaternion (norm {norm:.4f})')

        radius = math.sqrt(sum(v * v for v in position))
        if radius > self.config.max_radius_m:
            return self._block(
                f'position {radius:.1f} m from the origin exceeds max_radius_m '
                f'{self.config.max_radius_m:.1f} m')

        warning = ''
        if self._last_position is not None:
            gap = t_s - self._last_t
            if gap < 0.0:
                return self._block(f'LIO stamps went backwards by {-gap:.3f} s')
            if gap > self.config.max_gap_s:
                return self._block(
                    f'{gap:.2f} s gap in the LIO stream exceeds max_gap_s '
                    f'{self.config.max_gap_s:.2f} s; a restarted FAST-LIO has a new origin')
            step = math.dist(position, self._last_position)
            if step > self.config.max_step_m:
                return self._block(
                    f'position jumped {step:.2f} m in {gap:.3f} s, exceeding max_step_m '
                    f'{self.config.max_step_m:.2f} m')
            if step > self.config.warn_step_m:
                warning = f'position step {step * 100:.0f} cm'
                self.counts.warned += 1

        self._last_t = t_s
        self._last_position = position
        self.state = State.FORWARDING
        self.counts.forwarded += 1
        return Decision(True, self.state, '', warning)

    def _block(self, reason: str) -> Decision:
        self.state = State.BLOCKED
        self.reason = reason
        self.counts.rejected += 1
        return Decision(False, self.state, reason)

    def status(self) -> str:
        """One-line status for the node's status topic and logs."""
        text = (f'{self.state.value} forwarded={self.counts.forwarded} '
                f'rejected={self.counts.rejected} warned={self.counts.warned}')
        return f'{text} reason={self.reason}' if self.reason else text
