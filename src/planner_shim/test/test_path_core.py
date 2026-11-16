# filepath: src/planner_shim/test/test_path_core.py
import math

import numpy as np
import pytest

from planner_shim.shim_core import EnuBox
from planner_shim.path_core import (
    FREE, OCC, UNK, SliceGrid, traversable, points_free, snap, astar,
    line_clear, shortcut, plan_path, path_clear, remaining_waypoints,
    stop_reason,
)


def room(nx=30, ny=20):
    """Free room with a 1-cell wall border; clearance from a simple transform."""
    occ = np.full((nx, ny), FREE, dtype=np.uint8)
    occ[0, :] = occ[-1, :] = OCC
    occ[:, 0] = occ[:, -1] = OCC
    return occ


def clearance(occ, res=0.2):
    from scipy import ndimage
    return (ndimage.distance_transform_edt(occ != OCC) * res).astype(np.float32)


def grid_for(shape, res=0.2):
    nx, ny = shape
    return SliceGrid(res, 0, nx - 1, 0, ny - 1)


class TestGrid:
    def test_default_box_is_46_by_46(self):
        g = SliceGrid.from_box(EnuBox(-4.5, 4.5, -4.5, 4.5, 0.2, 2.3), 0.2)
        assert (g.nx, g.ny) == (46, 46)
        assert g.center(0, 0) == pytest.approx((-4.5, -4.5))
        assert g.layer_center(1.2) == pytest.approx(1.3)   # 1.2/0.2 float trap

    def test_cell_round_trip_and_outside(self):
        g = SliceGrid.from_box(EnuBox(-4.5, 4.5, -4.5, 4.5, 0.2, 2.3), 0.2)
        for i, j in [(0, 0), (45, 45), (10, 31)]:
            assert g.cell(*g.center(i, j)) == (i, j)
        assert g.cell(9.0, 0.0) is None

    def test_slice_points_order_is_i_outer_j_inner(self):
        g = SliceGrid(0.2, 0, 1, 0, 2)
        pts = g.slice_points(1.3)
        assert len(pts) == 6
        assert pts[1][:2] == pytest.approx(g.center(0, 1))
        assert all(p[2] == pytest.approx(1.3) for p in pts)

    def test_column_points_cover_every_layer(self):
        g = SliceGrid(0.2, -5, 5, -5, 5)
        zs = [p[2] for p in g.column_points(0.1, 0.1, 0.5, 1.3)]
        assert zs == pytest.approx([0.5, 0.7, 0.9, 1.1, 1.3])
        assert g.column_points(50.0, 0.0, 0.5, 1.3) == []


class TestTraversable:
    def test_unknown_is_never_traversable_whatever_its_clearance(self):
        occ = np.array([[UNK, FREE]], dtype=np.uint8)
        clr = np.array([[4.0, 4.0]], dtype=np.float32)
        assert traversable(occ, clr, 0.4).tolist() == [[False, True]]

    def test_clearance_and_negative_clearance(self):
        occ = np.zeros((1, 3), dtype=np.uint8)
        clr = np.array([[0.39, 0.4, -1.0]], dtype=np.float32)
        assert traversable(occ, clr, 0.4).tolist() == [[False, True, False]]

    def test_points_free(self):
        assert points_free([FREE, FREE], [1.0, 0.5], 0.4)
        assert not points_free([FREE, UNK], [1.0, 1.0], 0.4)
        assert not points_free([FREE], [0.2], 0.4)
        assert not points_free([], [], 0.4)


class TestAstar:
    def test_straight_line_in_open_room(self):
        occ = room()
        trav = traversable(occ, clearance(occ), 0.4)
        path = astar(trav, (5, 10), (20, 10))
        assert path[0] == (5, 10) and path[-1] == (20, 10)
        assert len(path) == 16

    def test_goes_round_a_wall_and_never_cuts_a_corner(self):
        trav = np.ones((9, 9), dtype=bool)
        trav[4, 0:7] = False                       # wall with a gap at the top
        path = astar(trav, (1, 1), (7, 1))
        assert path is not None
        for (a, b), (c, d) in zip(path[:-1], path[1:]):
            assert max(abs(c - a), abs(d - b)) == 1
            if a != c and b != d:
                assert trav[c, b] and trav[a, d]
        assert all(trav[c] for c in path)

    def test_diagonal_gap_is_not_passable(self):
        trav = np.ones((4, 4), dtype=bool)
        trav[1, 0] = trav[0, 1] = False            # start boxed in diagonally
        assert astar(trav, (0, 0), (3, 3)) is None

    def test_unreachable_and_blocked_ends(self):
        trav = np.ones((5, 5), dtype=bool)
        trav[2, :] = False
        assert astar(trav, (0, 0), (4, 4)) is None
        assert astar(trav, (2, 2), (4, 4)) is None
        assert astar(trav, (0, 0), (0, 0)) == [(0, 0)]

    def test_deterministic(self):
        occ = room()
        trav = traversable(occ, clearance(occ), 0.4)
        assert astar(trav, (4, 4), (25, 15)) == astar(trav, (4, 4), (25, 15))


class TestShortcut:
    def test_open_room_collapses_to_two_points(self):
        occ = room()
        trav = traversable(occ, clearance(occ), 0.4)
        path = astar(trav, (4, 4), (25, 15))
        assert shortcut(path, trav) == [(4, 4), (25, 15)]

    def test_shortcut_never_crosses_blocked_cells(self):
        trav = np.ones((12, 12), dtype=bool)
        trav[6, 0:9] = False
        path = astar(trav, (2, 2), (10, 2))
        sc = shortcut(path, trav)
        assert sc[0] == (2, 2) and sc[-1] == (10, 2) and len(sc) >= 3
        for a, b in zip(sc[:-1], sc[1:]):
            assert line_clear(trav, a, b)

    def test_line_clear_rejects_a_diagonal_squeeze(self):
        trav = np.ones((3, 3), dtype=bool)
        trav[1, 0] = False
        assert not line_clear(trav, (0, 0), (2, 1))


class TestPlanPath:
    def setup_method(self):
        self.occ = room()
        self.clr = clearance(self.occ)
        self.grid = grid_for(self.occ.shape)

    def test_plans_and_ends_on_the_goal_cell_centre(self):
        wps, why = plan_path(self.grid, self.occ, self.clr,
                             self.grid.center(5, 10), self.grid.center(20, 10), 0.4, 0.6)
        assert why == ''
        assert wps[-1] == pytest.approx(self.grid.center(20, 10))

    def test_goal_in_unknown_is_refused(self):
        occ = self.occ.copy()
        occ[18:, :] = UNK
        wps, why = plan_path(self.grid, occ, self.clr,
                             self.grid.center(5, 10), self.grid.center(20, 10), 0.4, 0.6)
        assert wps is None and 'goal cell' in why

    def test_path_never_enters_unknown(self):
        occ = self.occ.copy()
        occ[10:14, 3:17] = UNK                     # unknown slab across the middle
        wps, why = plan_path(self.grid, occ, self.clr,
                             self.grid.center(5, 10), self.grid.center(20, 10), 0.4, 0.6)
        assert why == ''
        trav = traversable(occ, self.clr, 0.4)
        prev = self.grid.center(5, 10)
        for w in wps:
            a = [(v - 0.1) / 0.2 for v in prev]
            b = [(v - 0.1) / 0.2 for v in w]
            assert line_clear(trav, a, b)
            prev = w

    def test_wall_with_no_gap_gives_no_path(self):
        occ = self.occ.copy()
        occ[12, :] = OCC
        clr = clearance(occ)
        wps, why = plan_path(self.grid, occ, clr,
                             self.grid.center(5, 10), self.grid.center(20, 10), 0.4, 0.6)
        assert wps is None and 'no path' in why

    def test_vehicle_outside_or_boxed_in(self):
        wps, why = plan_path(self.grid, self.occ, self.clr, (99.0, 0.0),
                             self.grid.center(20, 10), 0.4, 0.6)
        assert wps is None and 'outside' in why
        wps, why = plan_path(self.grid, self.occ, self.clr, self.grid.center(0, 0),
                             self.grid.center(20, 10), 0.4, 0.3)
        assert wps is None and 'not in traversable' in why

    def test_start_equals_goal(self):
        wps, why = plan_path(self.grid, self.occ, self.clr,
                             self.grid.center(5, 10), self.grid.center(5, 10), 0.4, 0.6)
        assert why == '' and wps == [pytest.approx(self.grid.center(5, 10))]


class TestPathClear:
    def setup_method(self):
        self.occ = room()
        self.clr = clearance(self.occ)
        self.grid = grid_for(self.occ.shape)
        self.robot = self.grid.center(5, 10)
        self.wps = [self.grid.center(20, 10)]

    def test_clear_path(self):
        assert path_clear(self.grid, self.occ, self.clr, self.robot, self.wps, 0.4, 0.3)

    def test_new_obstacle_on_the_path_blocks(self):
        occ = self.occ.copy()
        occ[12, 9:12] = OCC
        assert not path_clear(self.grid, occ, clearance(occ), self.robot, self.wps, 0.4, 0.3)

    def test_obstacle_at_the_vehicle_itself_is_ignored(self):
        occ = self.occ.copy()
        clr = self.clr.copy()
        clr[5, 10] = 0.1
        assert path_clear(self.grid, occ, clr, self.robot, self.wps, 0.4, 0.3)

    def test_remaining_waypoints(self):
        wps = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        assert remaining_waypoints(wps, (0.0, 0.0), 0.3) == wps
        assert remaining_waypoints(wps, (1.1, 0.0), 0.3) == wps[1:]
        assert remaining_waypoints(wps, (3.0, 0.0), 0.3) == wps     # last one is kept


class TestStopReason:
    ok = dict(pose_t=9.9, pose_timeout_s=0.5, lio_state=0, lio_t=9.5,
              lio_timeout_s=3.0, platform_t=9.5, platform_timeout_s=2.0,
              offboard=True, flying=True)

    def test_all_good(self):
        assert stop_reason(10.0, **self.ok) == ''

    @pytest.mark.parametrize('change,word', [
        (dict(pose_t=None), 'pose'), (dict(pose_t=9.0), 'pose'),
        (dict(lio_t=None), 'LIO health unknown'), (dict(lio_t=5.0), 'LIO health unknown'),
        (dict(lio_state=1), 'LIO degraded'), (dict(lio_state=None), 'LIO degraded'),
        (dict(platform_t=None), 'platform info'), (dict(platform_t=7.0), 'platform info'),
        (dict(offboard=False), 'Offboard'), (dict(flying=False), 'FLYING'),
    ])
    def test_each_fault(self, change, word):
        assert word in stop_reason(10.0, **{**self.ok, **change})
