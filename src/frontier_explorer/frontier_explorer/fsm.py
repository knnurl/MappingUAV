# filepath: src/frontier_explorer/frontier_explorer/fsm.py
"""Mission state machine and interlocks (spec §9), with the planning cycle
wrapper. No ROS imports: the node feeds Inputs and executes the returned
actions, tests do the same with a fake clock.

Actions returned by every method (list of tuples):
  ('goal', (x, y, z), kind)   kind: 'explore' | 'hold' | 'return'
  ('hover',)                  hover request to the planner
  ('alert', text)

Additions to spec v3.1 (operator, 2026-11-16):
  I10 battery: low => DONE(BATTERY), flying back to the start pose when that
      is allowed; unknown or stale while required => PAUSED.
  I11 px4_odom_bridge not FORWARDING or silent => PAUSED with no goal and no
      hover: the watchdog owns that response (it lands).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from planner_shim.shim_core import check_goal

from . import lifecycle as lc
from .grid import FREE
from .planner import plan, utility_of_goal

IDLE, WAIT_MAP, SELECTING, NAVIGATING, PAUSED, DONE, FAULT = range(7)
NAMES = ['IDLE', 'WAIT_MAP', 'SELECTING', 'NAVIGATING', 'PAUSED', 'DONE', 'FAULT']
ACTIVE = (WAIT_MAP, SELECTING, NAVIGATING)

LIO_OK, FENCE_OK, BRIDGE_FORWARDING = 0, 0, 1

# id: (target state, hold goal, hover request)   spec §9.3
ROWS = {
    'I1': (PAUSED, False, True),
    'I2': (PAUSED, False, False),
    'I3': (PAUSED, False, False),
    'I4': (PAUSED, False, False),
    'I5': (PAUSED, True, True),
    'I6': (PAUSED, False, True),
    'I7': (FAULT, False, True),
    'I8': (FAULT, False, True),
    'I9': (PAUSED, True, True),
    'I10': (PAUSED, True, True),
    'I11': (PAUSED, False, False),
}
ORDER = ['I1', 'I2', 'I3', 'I4', 'I5', 'I6', 'I7', 'I8', 'I9', 'I10', 'I11']
NO_COMPETE = ('I3', 'I4', 'I11')     # SR-10: someone else owns the response


@dataclass
class Inputs:
    """Everything the state machine may look at. Times are seconds on the
    caller's clock; None means never received."""
    now: float
    pose: tuple = None
    pose_t: float = None
    platform_t: float = None
    connected: bool = False
    armed: bool = False
    offboard: bool = False
    flying: bool = False
    lio_state: int = None
    lio_t: float = None
    fence_state: int = None
    fence_t: float = None
    bridge_state: int = None
    bridge_t: float = None
    battery: float = None            # fraction 0..1
    battery_t: float = None
    frame_diverged: bool = False
    frame_valid: bool = False        # at least one AS2/LIO sample pair compared
    map_available: bool = True


@dataclass
class Cycle:
    """Result of one planning cycle (spec §7.1)."""
    ok: bool
    free_cells: int = 0
    stuck: bool = False
    goal_cell: tuple = None
    goal_xyz: tuple = None
    utility: float = -math.inf
    tight: bool = False
    path_len: float = math.inf
    visible: list = field(default_factory=list)
    n_clusters: int = 0
    frontier_cells: int = 0
    unexplored: list = field(default_factory=list)
    front: object = None
    cur_utility: float = None        # U_cur of the active goal, None if not a candidate
    cur_dist: float = math.inf       # path distance to the active goal now
    plan_ms: float = 0.0


def run_cycle(p, grid, occ, clr, pose_xy, blacklist, current_goal_cell=None):
    """plan() plus what the lifecycle and the commitment rule need."""
    dec = plan(grid, occ, clr, pose_xy, blacklist, p)
    front = dec.masks['front']
    cyc = Cycle(ok=True, free_cells=int((occ == FREE).sum()), stuck=dec.stuck,
                n_clusters=dec.n_clusters, frontier_cells=int(front.sum()), front=front)
    cyc.unexplored = [
        {'cells': int(len(C)), 'centroid': [
            float(np.mean([grid.center(i, j)[0] for i, j in C])),
            float(np.mean([grid.center(i, j)[1] for i, j in C]))]}
        for _, C in dec.exhausted]
    if dec.stuck:
        return cyc
    dist = dec.masks['dist']
    if dec.goal is not None:
        x, y = grid.center(*dec.goal)
        cyc.goal_cell = tuple(dec.goal)
        cyc.goal_xyz = (x, y, grid.z_L)
        cyc.utility = float(dec.utility)
        cyc.tight = dec.tight
        cyc.path_len = float(dist[dec.goal])
        cyc.visible = [tuple(int(v) for v in c) for c in dec.visible]
    if current_goal_cell is not None:
        cyc.cur_dist = float(dist[tuple(current_goal_cell)])
        cyc.cur_utility = utility_of_goal(
            tuple(current_goal_cell), grid, occ, clr, dec, blacklist, p)
    return cyc


class Mission:
    def __init__(self, p):
        self.p = p
        self.state = IDLE
        self.reason = ''
        self.raised = []
        self.blacklist = lc.Blacklist(p.blacklist_max_entries, p.blacklist_ttl_s)
        self.life = lc.GoalLifecycle(p)
        self.goal_cell = None
        self.goal_meta = {}
        self.start_pose = None
        self.t_start = None
        self.empty_cycles = 0
        self.map_fail = 0
        self.stuck = False
        self.t_wait_map = None
        self.unexplored = []
        self.last = Cycle(ok=False)

    # ------------------------------------------------------- interlocks
    def interlocks(self, i: Inputs):
        p, r = self.p, []

        def stale(t, limit):
            return t is None or i.now - t > limit

        if i.pose is None or stale(i.pose_t, p.pose_timeout_s):
            r.append('I1')
        if (stale(i.platform_t, p.platform_info_timeout_s) or not i.connected
                or not i.armed or not i.offboard or not i.flying):
            r.append('I2')
        if stale(i.lio_t, p.lio_heartbeat_timeout_s) or i.lio_state != LIO_OK:
            r.append('I3')
        if stale(i.fence_t, p.fence_heartbeat_timeout_s) or i.fence_state != FENCE_OK:
            r.append('I4')
        if self.map_fail >= p.map_fail_max:
            r.append('I5')
        if i.frame_diverged:
            r.append('I6')
        if self.stuck:
            r.append('I9')
        if p.battery_required and (i.battery is None
                                   or stale(i.battery_t, p.battery_timeout_s)):
            r.append('I10')
        if (stale(i.bridge_t, p.bridge_heartbeat_timeout_s)
                or i.bridge_state != BRIDGE_FORWARDING):
            r.append('I11')
        return r

    def battery_low(self, i: Inputs):
        return i.battery is not None and i.battery < self.p.battery_min_fraction

    def preconditions(self, i: Inputs, config_errors):
        """START preconditions P1-P7 (+ battery, bridge). Returns failures."""
        raised = self.interlocks(i)
        f = [f'P1 {e}' for e in config_errors]
        if 'I2' in raised:
            f.append('P2 platform not connected, armed, in Offboard and FLYING')
        if 'I1' in raised:
            f.append('P3 no fresh pose')
        if 'I3' in raised:
            f.append('P4 LIO health not OK')
        if 'I4' in raised:
            f.append('P5 geofence watchdog not OK')
        if not i.map_available:
            f.append('P6 map_interface/query not available')
        if i.frame_diverged or not i.frame_valid:
            f.append('P7 frame consistency not established')
        if 'I10' in raised:
            f.append('P8 no fresh battery state')
        elif self.battery_low(i):
            f.append('P8 battery below battery_min_fraction')
        if 'I11' in raised:
            f.append('P9 px4_odom_bridge not FORWARDING')
        return f

    # --------------------------------------------------------- helpers
    def _set(self, state, reason, actions):
        if (state, reason) != (self.state, self.reason):
            actions.append(('alert', f'{NAMES[state]}: {reason}' if reason else NAMES[state]))
        self.state, self.reason = state, reason

    def _drop_goal(self):
        self.life.clear()
        self.goal_cell = None
        self.goal_meta = {}

    def hold_goal(self, i: Inputs, actions, target=None, kind='hold'):
        """§8.6. Publishes a hold (or return) goal only when every condition
        holds; otherwise says why not."""
        p = self.p
        why = None
        if any(k in self.raised for k in NO_COMPETE):
            why = 'LIO, geofence or bridge response in progress'
        elif not i.flying:
            why = 'platform not FLYING'
        elif i.pose is None or i.pose_t is None or i.now - i.pose_t > p.pose_timeout_s:
            why = 'no fresh pose'
        else:
            if target is None:
                m = p.obstacle_inflation + 0.05
                target = (min(max(i.pose[0], p.x_min + m), p.x_max - m),
                          min(max(i.pose[1], p.y_min + m), p.y_max - m),
                          min(max(i.pose[2], p.z_min + m), p.z_max - m))
            ok, msg = check_goal(target, p.box, p.obstacle_inflation)
            if not ok:
                why = msg
        if why:
            actions.append(('alert', f'{kind} goal not published: {why}'))
            return False
        actions.append(('goal', tuple(target), kind))
        return True

    def _finish(self, i: Inputs, reason, actions, force_return=False):
        self._drop_goal()
        self._set(DONE, reason, actions)
        if (force_return or self.p.on_done == 'return_start') and self.start_pose:
            if self.hold_goal(i, actions, target=self.start_pose, kind='return'):
                return
        self.hold_goal(i, actions)

    def _raise(self, i: Inputs, ids, actions):
        """T15: move to PAUSED or FAULT for the raised interlocks."""
        fault = [k for k in ids if ROWS[k][0] == FAULT]
        ordered = sorted(ids, key=ORDER.index)
        lead = (fault or ordered)[0]
        self._drop_goal()
        self._set(ROWS[lead][0], lead, actions)
        if ROWS[lead][1]:
            self.hold_goal(i, actions)
        if any(ROWS[k][2] for k in ids) and not any(k in ids for k in NO_COMPETE):
            actions.append(('hover',))

    def fault(self, i: Inputs, interlock, detail=''):
        """I7 (pre-validated goal refused) or I8 (internal error)."""
        actions = []
        if self.state == FAULT:
            return actions
        self.raised = sorted(set(self.raised) | {interlock}, key=ORDER.index)
        self._drop_goal()
        self._set(FAULT, f'{interlock} {detail}'.strip(), actions)
        if not any(k in self.raised for k in NO_COMPETE):
            actions.append(('hover',))
        return actions

    # ------------------------------------------------- operator services
    def start(self, i: Inputs, config_errors):
        if self.state not in (IDLE, DONE):
            return False, f'start refused in state {NAMES[self.state]}', []
        self.raised = self.interlocks(i)
        failures = self.preconditions(i, config_errors)
        if failures:
            return False, '; '.join(failures), []
        actions = []
        self.blacklist.clear()
        self._drop_goal()
        self.empty_cycles = self.map_fail = 0
        self.stuck = False
        self.unexplored = []
        self.start_pose = tuple(i.pose)
        self.t_start = self.t_wait_map = i.now
        self._set(WAIT_MAP, '', actions)
        return True, 'mission started', actions

    def pause(self, i: Inputs):
        if self.state not in ACTIVE:
            return False, f'pause refused in state {NAMES[self.state]}', []
        actions = []
        self.raised = self.interlocks(i)
        self._drop_goal()
        self._set(PAUSED, 'OPERATOR', actions)
        self.hold_goal(i, actions)
        if not any(k in self.raised for k in NO_COMPETE):
            actions.append(('hover',))
        return True, 'paused', actions

    def resume(self, i: Inputs, config_errors):
        if self.state != PAUSED:
            return False, f'resume refused in state {NAMES[self.state]}', []
        self.raised = self.interlocks(i)
        blockers = list(self.raised) + [f'config: {e}' for e in config_errors]
        if self.battery_low(i):
            blockers.append('battery low')
        if blockers:
            return False, 'resume refused: ' + ', '.join(blockers), []
        actions = []
        self.map_fail = 0
        self.stuck = False
        self.empty_cycles = 0
        self._set(SELECTING, '', actions)
        return True, 'resumed', actions

    def stop(self, i: Inputs):
        if self.state == FAULT:
            return False, 'stop refused in FAULT: restart the node', []
        actions = []
        self.raised = self.interlocks(i)
        was_active = self.state in ACTIVE
        self._drop_goal()
        self._set(IDLE, '', actions)
        if was_active:
            self.hold_goal(i, actions)
        return True, 'stopped', actions

    # ----------------------------------------------------- monitor tick
    def tick(self, i: Inputs, planner_alerts=()):
        """10 Hz. Order (spec §9.3): interlocks, mission timeout, lifecycle."""
        actions = []
        self.raised = self.interlocks(i)
        if self.state not in ACTIVE:
            return actions
        if self.state == WAIT_MAP and i.now - self.t_wait_map > self.p.wait_map_timeout_s:
            self.map_fail = self.p.map_fail_max          # T4: per the I5 row
            self.raised = self.interlocks(i)
        if self.raised:
            self._raise(i, self.raised, actions)
            return actions
        if self.battery_low(i):
            self._finish(i, 'BATTERY', actions, force_return=True)
            return actions
        if i.now - self.t_start > self.p.max_mission_s:
            self._finish(i, 'TIMEOUT', actions)
            return actions
        if self.state != NAVIGATING:
            return actions
        for text in planner_alerts:
            ev = self.life.on_alert(i.now, text)
            if ev == lc.REJECTED_LIO:
                self.raised = sorted(set(self.raised) | {'I3'}, key=ORDER.index)
                self._raise(i, ['I3'], actions)
                return actions
            if ev == lc.REJECTED_OTHER:
                return actions + self.fault(i, 'I7', text)
        ev = self.life.tick(i.now, i.pose)
        if ev == lc.REACHED:
            gx, gy, _ = self.life.goal
            self.blacklist.add(i.now, gx, gy, self.p.visited_radius_m, lc.VISITED)
            self._drop_goal()
            self._set(SELECTING, '', actions)
        elif ev == lc.GOAL_TIMEOUT:
            self._fail_goal(i, 'GOAL_TIMEOUT', actions)
        elif ev == lc.NO_START:
            actions.append(('goal', self.life.goal, 'explore'))
        return actions

    def _fail_goal(self, i: Inputs, why, actions):
        gx, gy, _ = self.life.goal
        self.blacklist.add(i.now, gx, gy, self.p.blacklist_radius_m, lc.FAILED)
        actions.append(('alert', f'goal dropped: {why}'))
        self._drop_goal()
        self._set(SELECTING, '', actions)

    def on_planner_failed(self, i: Inputs, why):
        """Structured planner status: it gave up on the active goal."""
        actions = []
        if self.state == NAVIGATING and self.life.active:
            self._fail_goal(i, f'planner: {why}', actions)
        return actions

    # ---------------------------------------------------- planning cycle
    def wants_cycle(self):
        return self.state in ACTIVE

    def on_cycle(self, i: Inputs, cyc: Cycle):
        actions = []
        if self.state not in ACTIVE:
            return actions
        self.last = cyc
        if not cyc.ok:
            self.map_fail += 1
            return actions
        self.map_fail = 0
        self.stuck = cyc.stuck
        if cyc.stuck:
            return actions                    # I9 fires on the next tick
        if self.state == WAIT_MAP:
            if cyc.free_cells < self.p.map_ready_min_free_cells:
                return actions
            self._set(SELECTING, '', actions)
        if self.state == NAVIGATING:
            ev = self.life.on_cycle(i.now, cyc.cur_dist, cyc.front)
            if ev == lc.NO_PROGRESS:
                self._fail_goal(i, 'NO_PROGRESS', actions)
                return actions                # select with the new blacklist next cycle
            if ev == lc.FRONTIER_CLEARED:
                self._drop_goal()
                self._set(SELECTING, '', actions)
            elif cyc.goal_cell is not None and cyc.goal_cell != self.goal_cell:
                if cyc.cur_utility is None or \
                        cyc.utility > cyc.cur_utility + self.p.switch_margin:
                    self._publish(i, cyc, actions)        # T12 / re-select
                return actions
            else:
                return actions
        # SELECTING
        if cyc.goal_cell is None:
            self.empty_cycles += 1
            if self.empty_cycles >= self.p.done_confirm_cycles:
                self.unexplored = cyc.unexplored
                self._finish(i, 'COMPLETE', actions)
            return actions
        self.empty_cycles = 0
        self._publish(i, cyc, actions)
        return actions

    def _publish(self, i: Inputs, cyc: Cycle, actions):
        if not i.flying or i.pose is None:    # SR-4, belt and braces: I1/I2 cover it
            return
        self.life.start(i.now, cyc.goal_xyz, i.pose, cyc.path_len, cyc.visible)
        self.goal_cell = cyc.goal_cell
        self.goal_meta = {'utility': cyc.utility, 'tight': cyc.tight}
        actions.append(('goal', cyc.goal_xyz, 'explore'))
        self._set(NAVIGATING, '', actions)

    # ------------------------------------------------------------ status
    def status(self, now, extra=None):
        goal = None
        if self.life.active:
            x, y, z = self.life.goal
            goal = {'x': x, 'y': y, 'z': z, 'age_s': round(now - self.life.t_pub, 2),
                    'utility': self.goal_meta.get('utility'),
                    'tight': self.goal_meta.get('tight', False),
                    'republished': self.life.republished}
        s = {
            'state': NAMES[self.state], 'reason': self.reason,
            'mission_elapsed_s': round(now - self.t_start, 1) if self.t_start else 0.0,
            'goal': goal, 'clusters': self.last.n_clusters,
            'frontier_cells': self.last.frontier_cells,
            'known_free_m2': round(self.last.free_cells * self.p.resolution ** 2, 2),
            'blacklist': len(self.blacklist), 'empty_cycles': self.empty_cycles,
            'last_plan_ms': round(self.last.plan_ms, 1),
            'interlocks': list(self.raised), 'unexplored_clusters': self.unexplored,
        }
        s.update(extra or {})
        return s


class FrameMonitor:
    """SR-9 / §9.5: AS2 pose against LIO /Odometry position."""

    def __init__(self, p):
        self.p = p
        self.lio = []                # [(stamp, (x, y, z))], last 20
        self.bad_since = None
        self.diverged = False
        self.valid = False
        self.last_diff = None

    def on_lio(self, stamp, pos):
        self.lio.append((stamp, tuple(pos)))
        del self.lio[:-20]

    def on_pose(self, stamp, pos):
        if not self.lio:
            return
        t, lp = min(self.lio, key=lambda s: abs(s[0] - stamp))
        if abs(t - stamp) > self.p.frame_check_max_dt_s:
            return                    # a stamp gap says nothing about the frames
        self.valid = True
        self.last_diff = math.dist(pos, lp)
        if self.last_diff > self.p.frame_divergence_max_m:
            if self.bad_since is None:
                self.bad_since = stamp
            if stamp - self.bad_since >= self.p.frame_divergence_persist_s:
                self.diverged = True
        else:
            self.bad_since = None
            self.diverged = False
