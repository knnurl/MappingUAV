# filepath: src/frontier_explorer/test/test_fsm.py
"""Mission state machine and interlocks (spec §9), fake clock, no ROS."""
import dataclasses

import numpy as np
import pytest

import fixtures
from frontier_explorer import fsm
from frontier_explorer.config import Params
from frontier_explorer.fsm import (
    DONE, FAULT, IDLE, NAVIGATING, PAUSED, SELECTING, WAIT_MAP, Cycle,
    FrameMonitor, Inputs, Mission,
)


def good(now=0.0, **kw):
    base = dict(now=now, pose=(0.0, 0.0, 1.3), pose_t=now, platform_t=now,
                connected=True, armed=True, offboard=True, flying=True,
                lio_state=0, lio_t=now, fence_state=0, fence_t=now,
                bridge_state=1, bridge_t=now, battery=0.8, battery_t=now,
                frame_diverged=False, frame_valid=True, map_available=True)
    base.update(kw)
    return Inputs(**base)


def cyc(goal=(2.0, 0.0, 1.3), cell=(30, 22), free=500, **kw):
    front = np.zeros((46, 46), dtype=bool)
    front[10, 10] = True
    base = dict(ok=True, free_cells=free, goal_cell=cell, goal_xyz=goal, utility=3.0,
                path_len=2.0, visible=[(10, 10)], n_clusters=1, front=front)
    if goal is None:
        base.update(goal_cell=None, goal_xyz=None, visible=[])
    base.update(kw)
    return Cycle(**base)


def kinds(actions):
    return [a[0] if a[0] != 'goal' else f'goal:{a[2]}' for a in actions]


def navigating(p=None):
    m = Mission(p or Params())
    ok, msg, _ = m.start(good(), [])
    assert ok, msg
    acts = m.on_cycle(good(1.0), cyc())
    assert m.state == NAVIGATING and 'goal:explore' in kinds(acts)
    return m


# ----------------------------------------------------------- start (T-M07)
def test_start_goes_to_wait_map_then_selects_and_navigates():
    m = Mission(Params())
    ok, _, acts = m.start(good(), [])
    assert ok and m.state == WAIT_MAP and ('alert', 'WAIT_MAP') in acts
    assert m.on_cycle(good(1.0), cyc(free=10)) == [] and m.state == WAIT_MAP
    acts = m.on_cycle(good(3.0), cyc())
    assert m.state == NAVIGATING
    assert ('goal', (2.0, 0.0, 1.3), 'explore') in acts


@pytest.mark.parametrize('change,tag', [
    (dict(offboard=False), 'P2'), (dict(flying=False), 'P2'), (dict(armed=False), 'P2'),
    (dict(pose=None), 'P3'), (dict(lio_state=1), 'P4'), (dict(lio_t=None), 'P4'),
    (dict(fence_state=1), 'P5'), (dict(fence_t=None), 'P5'),
    (dict(map_available=False), 'P6'), (dict(frame_valid=False), 'P7'),
    (dict(frame_diverged=True), 'P7'), (dict(battery=None), 'P8'),
    (dict(battery=0.1), 'P8'), (dict(bridge_state=3), 'P9'), (dict(bridge_t=None), 'P9'),
])
def test_start_refused_for_each_failing_precondition(change, tag):
    m = Mission(Params())
    ok, msg, acts = m.start(good(**change), [])
    assert not ok and tag in msg and acts == [] and m.state == IDLE


def test_start_refused_on_config_errors_and_in_wrong_state():
    m = Mission(Params())
    ok, msg, _ = m.start(good(), ['C4: box outside the fence'])
    assert not ok and 'P1 C4' in msg
    m = navigating()
    assert not m.start(good(), [])[0]


def test_battery_not_required_lets_a_mission_start_without_the_topic():
    p = dataclasses.replace(Params(), battery_required=False)
    assert Mission(p).start(good(battery=None, battery_t=None), [])[0]


# ------------------------------------------------- interlocks (T-M02, T-M03)
@pytest.mark.parametrize('change,iid,hold,hover', [
    (dict(pose_t=-5.0), 'I1', False, True),
    (dict(offboard=False), 'I2', False, False),
    (dict(flying=False), 'I2', False, False),
    (dict(lio_state=1), 'I3', False, False),
    (dict(lio_t=-10.0), 'I3', False, False),
    (dict(fence_state=1), 'I4', False, False),
    (dict(fence_t=-10.0), 'I4', False, False),
    (dict(frame_diverged=True), 'I6', False, True),
    (dict(battery_t=-60.0), 'I10', True, True),
    (dict(bridge_state=3), 'I11', False, False),
    (dict(bridge_state=2), 'I11', False, False),
    (dict(bridge_t=-10.0), 'I11', False, False),
])
def test_each_interlock_pauses_with_its_row(change, iid, hold, hover):
    m = navigating()
    acts = m.tick(good(2.0, **change))
    assert m.state == PAUSED and m.reason == iid and iid in m.raised
    assert ('goal:hold' in kinds(acts)) is hold
    assert ('hover' in kinds(acts)) is hover
    assert not m.life.active


def test_map_failures_raise_i5_with_hold_and_hover():
    m = navigating()
    for k in range(3):
        m.on_cycle(good(2.0 + k), Cycle(ok=False))
    acts = m.tick(good(5.0))
    assert m.state == PAUSED and m.reason == 'I5'
    assert 'goal:hold' in kinds(acts) and 'hover' in kinds(acts)


def test_wait_map_timeout_pauses():                              # T4
    m = Mission(Params())
    m.start(good(), [])
    assert m.tick(good(29.0)) == []
    m.tick(good(31.0))
    assert m.state == PAUSED and m.reason == 'I5'


def test_stuck_raises_i9():
    m = navigating()
    m.on_cycle(good(2.0), Cycle(ok=True, free_cells=500, stuck=True))
    acts = m.tick(good(2.1))
    assert m.state == PAUSED and m.reason == 'I9' and 'goal:hold' in kinds(acts)


def test_lowest_id_is_the_reason_and_all_are_listed():
    m = navigating()
    m.tick(good(2.0, lio_state=1, frame_diverged=True, pose_t=-5.0))
    assert m.reason == 'I1' and m.raised == ['I1', 'I3', 'I6']


def test_no_hover_or_hold_while_lio_fence_or_bridge_own_the_response():   # SR-10
    m = navigating()
    acts = m.tick(good(2.0, lio_state=1, frame_diverged=True))
    assert kinds(acts) == ['alert']


def test_interlocks_tracked_in_paused_and_resume_gated():       # T-M05
    m = navigating()
    m.tick(good(2.0, lio_state=1))
    m.tick(good(3.0, lio_state=1))
    ok, msg, _ = m.resume(good(4.0, lio_state=1), [])
    assert not ok and 'I3' in msg and m.state == PAUSED
    ok, msg, _ = m.resume(good(5.0), ['C2: drift'])
    assert not ok and 'C2' in msg
    ok, _, _ = m.resume(good(6.0), [])
    assert ok and m.state == SELECTING


def test_blacklist_survives_pause_and_resume():
    m = navigating()
    m.tick(good(2.0, pose=(2.0, 0.0, 1.3)))                     # REACHED
    assert len(m.blacklist) == 1
    m.pause(good(3.0))
    m.resume(good(4.0), [])
    assert len(m.blacklist) == 1


# ------------------------------------------------------------ faults
def test_other_rejection_is_a_terminal_fault():                 # T14, T-M06
    m = navigating()
    acts = m.tick(good(1.5), ['goal REJECTED: goal outside planning box'])
    assert m.state == FAULT and m.reason.startswith('I7') and 'hover' in kinds(acts)
    assert not m.stop(good(2.0))[0]
    assert not m.start(good(2.0), [])[0]
    assert not m.resume(good(2.0), [])[0]
    assert m.tick(good(3.0)) == [] and m.state == FAULT


def test_lio_rejection_pauses_with_i3():                        # T13
    m = navigating()
    acts = m.tick(good(1.5), ['goal REJECTED: LIO degraded'])
    assert m.state == PAUSED and m.reason == 'I3' and kinds(acts) == ['alert']


def test_internal_error_faults():                               # T21
    m = navigating()
    acts = m.fault(good(2.0), 'I8', 'ValueError in plan')
    assert m.state == FAULT and 'hover' in kinds(acts)


def test_fault_beats_paused():
    m = navigating()
    m.raised = ['I1']
    m.fault(good(2.0, pose_t=-5.0), 'I7')
    assert m.state == FAULT and m.raised == ['I1', 'I7']


# ---------------------------------------------------- goal lifecycle paths
def test_reached_adds_a_visited_entry_and_selects_again():      # T8
    m = navigating()
    m.tick(good(2.0, pose=(2.0, 0.1, 1.3)))
    assert m.state == SELECTING
    assert m.blacklist.active(2.0) == [(2.0, 0.0, 0.6)]


def test_goal_timeout_adds_a_failed_entry():                    # T10
    m = navigating()
    m.tick(good(30.0, pose=(1.0, 0.0, 1.3)))
    assert m.state == SELECTING and m.blacklist.active(30.0) == [(2.0, 0.0, 1.0)]


def test_no_start_republishes_the_same_goal_once():             # T11
    m = navigating()
    acts = m.tick(good(7.0))
    assert acts == [('goal', (2.0, 0.0, 1.3), 'explore')]
    assert m.tick(good(8.0)) == []


def test_no_progress_fails_the_goal():
    m = navigating()
    m.on_cycle(good(10.0), cyc(cur_dist=2.0, cur_utility=3.0))
    m.on_cycle(good(20.0), cyc(cur_dist=2.0, cur_utility=3.0))
    assert m.state == SELECTING and len(m.blacklist) == 1


def test_frontier_cleared_selects_in_the_same_cycle():          # T9
    m = navigating()
    c = cyc(goal=(0.0, 2.0, 1.3), cell=(22, 32))
    c.front = np.zeros((46, 46), dtype=bool)                    # V_g gone
    acts = m.on_cycle(good(3.0), c)
    assert m.state == NAVIGATING and ('goal', (0.0, 2.0, 1.3), 'explore') in acts
    assert len(m.blacklist) == 0


def test_commitment():                                          # T12, FR-6
    m = navigating()
    other = dict(goal=(0.0, 2.0, 1.3), cell=(22, 32))
    assert m.on_cycle(good(3.0), cyc(**other, utility=4.9, cur_utility=3.0, cur_dist=1.5)) == []
    acts = m.on_cycle(good(5.0), cyc(**other, utility=5.1, cur_utility=3.0, cur_dist=1.2))
    assert ('goal', (0.0, 2.0, 1.3), 'explore') in acts and m.goal_cell == (22, 32)


def test_goal_no_longer_a_candidate_is_reselected_without_blacklisting():
    m = navigating()
    acts = m.on_cycle(good(3.0), cyc(goal=(0.0, 2.0, 1.3), cell=(22, 32),
                                     utility=0.5, cur_utility=None, cur_dist=1.5))
    assert 'goal:explore' in kinds(acts) and len(m.blacklist) == 0


def test_planner_failure_drops_the_goal_at_once():
    m = navigating()
    m.on_planner_failed(good(2.0), 'no path through observed free space')
    assert m.state == SELECTING and m.blacklist.active(2.0) == [(2.0, 0.0, 1.0)]


def test_no_goal_unless_flying():                               # SR-4, T-M04
    m = Mission(Params())
    m.start(good(), [])
    assert m.on_cycle(good(1.0, flying=False), cyc()) == [('alert', 'SELECTING')]
    assert m.state == SELECTING


# --------------------------------------------------------- termination
def test_done_after_confirm_cycles_holds():                     # T7, T-M08
    m = Mission(Params())
    m.start(good(), [])
    m.on_cycle(good(1.0), cyc(goal=None))
    m.on_cycle(good(3.0), cyc(goal=None))
    assert m.state == SELECTING
    acts = m.on_cycle(good(5.0), cyc(goal=None, unexplored=[{'cells': 4, 'centroid': [1, 1]}]))
    assert m.state == DONE and m.reason == 'COMPLETE' and 'goal:hold' in kinds(acts)
    assert m.unexplored == [{'cells': 4, 'centroid': [1, 1]}]
    assert m.start(good(6.0), [])[0]                            # DONE -> new mission


def test_a_goal_resets_the_empty_count():
    m = Mission(Params())
    m.start(good(), [])
    m.on_cycle(good(1.0), cyc(goal=None))
    m.on_cycle(good(3.0), cyc(goal=None))
    m.on_cycle(good(5.0), cyc())
    assert m.empty_cycles == 0 and m.state == NAVIGATING


def test_mission_timeout_and_return_start():
    p = dataclasses.replace(Params(), on_done='return_start')
    m = navigating(p)                                           # started at (0, 0, 1.3)
    acts = m.tick(good(901.0, pose=(1.0, 1.0, 1.3)))
    assert m.state == DONE and m.reason == 'TIMEOUT'
    assert ('goal', (0.0, 0.0, 1.3), 'return') in acts


def test_low_battery_ends_the_mission_and_flies_back():         # I10, D6
    m = navigating()
    acts = m.tick(good(50.0, battery=0.2, pose=(1.0, 0.5, 1.3)))
    assert m.state == DONE and m.reason == 'BATTERY'
    assert ('goal', (0.0, 0.0, 1.3), 'return') in acts


# ---------------------------------------------- operator services, SR-10
def test_pause_holds_and_hovers_stop_holds():                   # T17, T20
    m = navigating()
    ok, _, acts = m.pause(good(2.0))
    assert ok and m.state == PAUSED and m.reason == 'OPERATOR'
    assert kinds(acts) == ['alert', 'goal:hold', 'hover']
    ok, _, acts = m.stop(good(3.0))
    assert ok and m.state == IDLE and 'goal:hold' not in kinds(acts)   # was not active


@pytest.mark.parametrize('change', [dict(lio_state=1), dict(fence_state=2),
                                    dict(bridge_state=3)])
def test_stop_and_pause_publish_no_hold_goal_during_a_response(change):   # T-M09
    m = navigating()
    _, _, acts = m.stop(good(2.0, **change))
    assert 'goal:hold' not in kinds(acts)
    m = navigating()
    _, _, acts = m.pause(good(2.0, **change))
    assert 'goal:hold' not in kinds(acts) and 'hover' not in kinds(acts)


def test_hold_goal_is_clamped_inside_the_box():
    m = navigating()
    _, _, acts = m.pause(good(2.0, pose=(4.4, 0.0, 1.3)))
    hold = [a for a in acts if a[0] == 'goal'][0][1]
    assert hold[0] == pytest.approx(4.05)


def test_status_schema():
    m = navigating()
    s = m.status(5.0)
    for key in ('state', 'reason', 'mission_elapsed_s', 'goal', 'clusters',
                'frontier_cells', 'known_free_m2', 'blacklist', 'empty_cycles',
                'last_plan_ms', 'interlocks', 'unexplored_clusters'):
        assert key in s
    assert s['state'] == 'NAVIGATING' and s['goal']['x'] == 2.0


# ------------------------------------------------------- frame monitor
def test_frame_monitor():                                       # T-F06
    f = FrameMonitor(Params())
    f.on_pose(0.0, (0, 0, 0))
    assert not f.valid                                          # no LIO yet
    for k in range(30):
        t = k * 0.1
        f.on_lio(t, (0, 0, 0))
        f.on_pose(t, (0.6, 0, 0) if k >= 10 else (0.1, 0, 0))
        if k < 10:
            assert f.valid and not f.diverged
    assert f.diverged                                           # persisted > 1 s
    f.on_lio(3.0, (0, 0, 0))
    f.on_pose(3.0, (0.1, 0, 0))
    assert not f.diverged
    f.on_pose(9.0, (5, 5, 5))                                   # stamp gap: ignored
    assert not f.diverged


# ------------------------------ whole core, closed over the reveal fixture
def run_mission(true_occ, robot_cell, p):
    """Mission + run_cycle + lifecycle against the open-loop reveal world.
    The robot teleports to each goal. Not a simulator (spec §13.2)."""
    res = p.resolution
    world = fixtures.World(true_occ, res)
    box = fixtures.box_for(true_occ.shape, res)
    p = dataclasses.replace(p, x_min=box.x_min, x_max=box.x_max,
                            y_min=box.y_min, y_max=box.y_max)
    grid = p.grid()
    assert (grid.nx, grid.ny) == true_occ.shape
    robot = tuple(robot_cell)
    world.reveal(robot, p.view_range_m)
    m = Mission(p)
    t = 0.0
    pose = (*grid.center(*robot), grid.z_L)
    ok, msg, _ = m.start(good(t, pose=pose), [])
    assert ok, msg
    goals = []
    for _ in range(400):
        t += p.cycle_period_s
        occ, clr = world.observed()
        i = good(t, pose=pose)
        c = fsm.run_cycle(p, grid, occ, clr, pose[:2], m.blacklist.active(t), m.goal_cell)
        for a in m.on_cycle(i, c):
            if a[0] == 'goal' and a[2] == 'explore':
                goals.append(a[1])
                robot = grid.world_to_cell(a[1][0], a[1][1])
                pose = a[1]
                world.reveal(robot, p.view_range_m)
        m.tick(good(t + 0.1, pose=pose))
        if m.state in (DONE, PAUSED, FAULT):
            break
    return m, goals, world


def room_with_door(door_cells):
    occ = np.full((52, 18), 1, dtype=np.uint8)
    occ[3:23, 3:15] = 0
    occ[25:49, 3:15] = 0
    j0 = 9 - door_cells // 2
    occ[23:25, j0:j0 + door_cells] = 0
    return occ


def test_full_mission_explores_both_rooms_and_finishes():
    true = room_with_door(5)                                    # 1.0 m door
    m, goals, world = run_mission(true, (13, 9), Params())
    assert m.state == DONE and m.reason == 'COMPLETE', (m.state, m.reason)
    assert goals and m.unexplored == []
    assert world.known[25:49, 3:15].all()                       # room B fully seen


def test_full_mission_reports_what_it_could_not_reach():
    true = room_with_door(2)                                    # 0.4 m door
    m, goals, world = run_mission(true, (13, 9), Params())
    assert m.state == DONE and m.reason == 'COMPLETE'
    assert len(m.unexplored) >= 1
    assert not world.known[40:49, 3:15].all()


def test_enclosed_room_is_done_without_any_goal():
    true = np.full((24, 18), 1, dtype=np.uint8)
    true[3:21, 3:15] = 0
    m, goals, _ = run_mission(true, (12, 9), Params())
    assert m.state == DONE and goals == []
