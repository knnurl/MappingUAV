# filepath: src/frontier_explorer/test/test_lifecycle.py
import numpy as np

from frontier_explorer import lifecycle as lc
from frontier_explorer.config import Params


def started(path_len=4.0, visible=((5, 5),), p=None):
    life = lc.GoalLifecycle(p or Params())
    life.start(100.0, (2.0, 0.0, 1.3), (0.0, 0.0, 1.3), path_len, visible)
    return life


def front_with(*cells, shape=(10, 10)):
    m = np.zeros(shape, dtype=bool)
    for c in cells:
        m[c] = True
    return m


def test_reached_needs_xy_and_z():                              # T-L01
    life = started()
    assert life.tick(101.0, (1.75, 0.0, 1.3)) == lc.REACHED
    assert life.tick(101.0, (1.75, 0.0, 1.7)) is None
    assert life.tick(101.0, (1.6, 0.0, 1.3)) is None


def test_frontier_cleared_when_no_visible_cell_is_still_frontier():   # T-L02
    life = started(visible=((5, 5), (5, 6)))
    assert life.on_cycle(102.0, 3.9, front_with((5, 6))) is None
    assert life.on_cycle(104.0, 3.5, front_with((1, 1))) == lc.FRONTIER_CLEARED


def test_no_progress_and_progress_resets_the_window():          # T-L03
    life = started()
    front = front_with((5, 5))
    assert life.on_cycle(110.0, 4.0, front) is None
    assert life.on_cycle(114.0, 3.6, front) is None            # 0.4 m better: resets
    assert life.on_cycle(128.0, 3.5, front) is None            # 14 s since reset
    assert life.on_cycle(130.0, 3.5, front) == lc.NO_PROGRESS
    life = started()
    assert life.on_cycle(116.0, float('inf'), front) == lc.NO_PROGRESS   # unreachable


def test_timeout_formula_and_floor():                           # T-L04
    assert started(path_len=4.0).timeout == 24.0                # 3 * 4 / 0.5
    assert started(path_len=1.0).timeout == 20.0                # floor
    life = started(path_len=1.0)
    moving = (1.0, 0.0, 1.3)
    assert life.tick(119.9, moving) is None
    assert life.tick(120.1, moving) == lc.GOAL_TIMEOUT


def test_no_start_republishes_exactly_once():                   # T-L05
    life = started()
    still = (0.1, 0.0, 1.3)
    assert life.tick(104.9, still) is None
    assert life.tick(105.1, still) == lc.NO_START
    assert life.republished
    assert life.tick(106.0, still) is None
    moved = started()
    assert moved.tick(105.1, (0.5, 0.0, 1.3)) is None


def test_blacklist_fifo_ttl_and_clear():                        # T-L06
    b = lc.Blacklist(max_entries=2, ttl_s=0.0)
    b.add(0.0, 1, 1, 0.6, lc.VISITED)
    b.add(1.0, 2, 2, 1.0, lc.FAILED)
    b.add(2.0, 3, 3, 1.0, lc.FAILED)
    assert b.active(1e6) == [(2.0, 2.0, 1.0), (3.0, 3.0, 1.0)]
    t = lc.Blacklist(max_entries=8, ttl_s=10.0)
    t.add(0.0, 1, 1, 0.6, lc.VISITED)
    assert t.active(10.0) and not t.active(10.1)
    b.clear()
    assert len(b) == 0


def test_alert_matching():                                      # T-L07, T-N06 strings
    life = started()
    assert life.on_alert(100.5, 'goal REJECTED: LIO degraded') == lc.REJECTED_LIO
    assert life.on_alert(100.5, 'goal REJECTED: goal (9.00,0.00,1.30) outside planning '
                                'box with 0.4 m inflation margin') == lc.REJECTED_OTHER
    assert life.on_alert(100.5, 'path blocked: replanning (1/3)') is None
    assert life.on_alert(101.5, 'goal REJECTED: LIO degraded') is None    # outside window
    assert lc.GoalLifecycle(Params()).on_alert(0.0, 'goal REJECTED: x') is None
