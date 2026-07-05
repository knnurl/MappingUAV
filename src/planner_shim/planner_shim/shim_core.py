# filepath: src/planner_shim/planner_shim/shim_core.py
"""Pure logic for the go_to_with_avoidance wrapper (WP-D): goal and
trajectory validity checks. Unit-tested now per masterplan §6 ("config
parsing, map-query adapters, trajectory validity checks are unit-tested
now"); the closed planner loop stays BUILT_UNVERIFIED.

Frames [FIXED]: odom/ENU throughout. The ENU box here is the PLANNING box —
derived from (inside) the geofence; the NED geofence watchdog remains the
independent enforcement layer.
"""
import math
from dataclasses import dataclass


@dataclass
class EnuBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def __post_init__(self):
        if not (self.x_min < self.x_max and self.y_min < self.y_max
                and self.z_min < self.z_max):
            raise ValueError('box min must be < max on every axis')

    def contains(self, x, y, z, margin=0.0):
        return (self.x_min + margin <= x <= self.x_max - margin and
                self.y_min + margin <= y <= self.y_max - margin and
                self.z_min + margin <= z <= self.z_max - margin)


@dataclass
class Limits:
    max_vel: float
    max_acc: float

    def __post_init__(self):
        if self.max_vel <= 0 or self.max_acc <= 0:
            raise ValueError('limits must be positive')


def check_goal(goal, box: EnuBox, inflation: float):
    """Goal admissibility: finite, inside the planning box by >= inflation.
    Returns (ok, reason)."""
    x, y, z = goal
    if not all(math.isfinite(v) for v in goal):
        return False, 'goal has non-finite coordinates'
    if not box.contains(x, y, z, margin=inflation):
        return False, (f'goal ({x:.2f},{y:.2f},{z:.2f}) outside planning box '
                       f'with {inflation} m inflation margin')
    return True, ''


def check_trajectory(samples, box: EnuBox, limits: Limits):
    """Validate a time-sampled trajectory [(t, x, y, z), ...]:
    monotonic time, finite values, containment, velocity and acceleration
    within limits. Returns (ok, reason)."""
    if len(samples) < 2:
        return False, 'trajectory has fewer than 2 samples'
    vels = []
    for i, (t, x, y, z) in enumerate(samples):
        if not all(math.isfinite(v) for v in (t, x, y, z)):
            return False, f'non-finite value at sample {i}'
        if not box.contains(x, y, z):
            return False, (f'sample {i} ({x:.2f},{y:.2f},{z:.2f}) '
                           'leaves the planning box')
        if i > 0:
            t0, x0, y0, z0 = samples[i - 1]
            dt = t - t0
            if dt <= 0:
                return False, f'non-monotonic time at sample {i}'
            v = math.dist((x, y, z), (x0, y0, z0)) / dt
            if v > limits.max_vel * 1.05:   # 5% numeric slack
                return False, (f'velocity {v:.2f} m/s at sample {i} exceeds '
                               f'{limits.max_vel} m/s')
            vels.append((t0 + dt / 2, v))
    for j in range(1, len(vels)):
        tm0, v0 = vels[j - 1]
        tm1, v1 = vels[j]
        dt = tm1 - tm0
        if dt > 0 and abs(v1 - v0) / dt > limits.max_acc * 1.10:
            return False, (f'acceleration {abs(v1 - v0) / dt:.2f} m/s^2 near '
                           f'sample {j} exceeds {limits.max_acc} m/s^2')
    return True, ''
