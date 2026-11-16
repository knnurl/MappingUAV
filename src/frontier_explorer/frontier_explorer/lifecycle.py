# filepath: src/frontier_explorer/frontier_explorer/lifecycle.py
"""Goal lifecycle (spec §8) and the blacklist (§8.5). No ROS imports; every
method takes the time, so tests drive the clock."""
import math
from collections import deque

REACHED = 'REACHED'
FRONTIER_CLEARED = 'FRONTIER_CLEARED'
NO_PROGRESS = 'NO_PROGRESS'
GOAL_TIMEOUT = 'GOAL_TIMEOUT'
NO_START = 'NO_START'
REJECTED_LIO = 'REJECTED_LIO'
REJECTED_OTHER = 'REJECTED_OTHER'

VISITED, FAILED = 'VISITED', 'FAILED'

REJECT_PREFIX = 'goal REJECTED'
REJECT_LIO_PREFIX = 'goal REJECTED: LIO degraded'


class Blacklist:
    """FIFO of (x, y, radius, reason, t_added). ttl_s == 0 keeps entries for
    the whole mission."""

    def __init__(self, max_entries, ttl_s):
        self.entries = deque(maxlen=max_entries)
        self.ttl_s = ttl_s

    def add(self, now, x, y, radius, reason):
        self.entries.append((float(x), float(y), float(radius), reason, now))

    def active(self, now):
        """[(x, y, radius)] of the entries still in force."""
        return [(x, y, r) for x, y, r, _, t in self.entries
                if self.ttl_s == 0 or now - t <= self.ttl_s]

    def clear(self):
        self.entries.clear()

    def __len__(self):
        return len(self.entries)


class GoalLifecycle:
    def __init__(self, p):
        self.p = p
        self.goal = None

    @property
    def active(self):
        return self.goal is not None

    def start(self, now, goal, pose, path_len, visible_cells):
        """§8.1. goal, pose: (x, y, z). visible_cells: iterable of (i, j), the
        sample frontier cells visible from the goal (V_g)."""
        p = self.p
        self.goal = tuple(goal)
        self.t_pub = now
        self.pose_at_pub = tuple(pose)
        self.path_len = float(path_len)
        self.timeout = max(p.goal_timeout_min_s,
                           p.goal_timeout_factor * self.path_len / p.nominal_speed_m_s)
        self.best_dist = self.path_len
        self.t_best = now
        self.republished = False
        self.visible = [tuple(int(v) for v in c) for c in visible_cells]

    def clear(self):
        self.goal = None

    def tick(self, now, pose):
        """Fast checks (10 Hz). Returns one event or None."""
        if self.goal is None or pose is None:
            return None
        p = self.p
        gx, gy, gz = self.goal
        if (math.hypot(pose[0] - gx, pose[1] - gy) <= p.reach_tolerance_xy_m
                and abs(pose[2] - gz) <= p.reach_tolerance_z_m):
            return REACHED
        if now - self.t_pub > self.timeout:
            return GOAL_TIMEOUT
        if (not self.republished and now - self.t_pub > p.start_motion_timeout_s
                and math.dist(pose, self.pose_at_pub) < p.progress_min_m):
            self.republished = True
            return NO_START
        return None

    def on_cycle(self, now, dist_now, front):
        """Per planning cycle. dist_now: path distance from the robot to the
        goal cell this cycle (inf if unreachable). front: frontier mask."""
        if self.goal is None:
            return None
        if self.visible and not any(front[c] for c in self.visible):
            return FRONTIER_CLEARED
        if dist_now < self.best_dist - self.p.progress_min_m:
            self.best_dist = dist_now
            self.t_best = now
        elif now - self.t_best > self.p.progress_window_s:
            return NO_PROGRESS
        return None

    def on_alert(self, now, text):
        """§8.4: a planner alert within reject_window_s of the publish."""
        if self.goal is None or not text.startswith(REJECT_PREFIX):
            return None
        if not (0.0 <= now - self.t_pub <= self.p.reject_window_s):
            return None
        return REJECTED_LIO if text.startswith(REJECT_LIO_PREFIX) else REJECTED_OTHER
