# filepath: src/frontier_explorer/test/fixtures.py
"""Open-loop reveal fixtures for scenario tests (spec §13.2, Appendix A).
NOT a simulator: the robot teleports to each goal and the true map is revealed
by raycasting. It makes no claim about vehicle behaviour.
"""
import math

import numpy as np
from scipy import ndimage

from planner_shim.shim_core import EnuBox
from frontier_explorer.grid import EPS, OCC, UNK, Grid
from frontier_explorer.planner import plan

class World:
    """True band map + known mask. Reveal = octomap-like raycasting: along each ray
    to every cell within view range, cells become known up to and including the
    first occupied cell."""

    def __init__(self, true_occ, res=0.2, clearance_cap=4.0):
        self.true = true_occ.astype(np.uint8)
        self.known = np.zeros(true_occ.shape, dtype=bool)
        self.res = res
        self.cap = clearance_cap

    def reveal(self, cell, view_range):
        nx, ny = self.true.shape
        r = int(math.ceil(view_range / self.res))
        i0, j0 = cell
        ii, jj = np.meshgrid(np.arange(max(0, i0 - r), min(nx, i0 + r + 1)),
                             np.arange(max(0, j0 - r), min(ny, j0 + r + 1)), indexing='ij')
        f = np.stack([ii.ravel(), jj.ravel()], axis=1).astype(float)
        d = np.hypot(f[:, 0] - i0, f[:, 1] - j0)
        f, d = f[d * self.res <= view_range + EPS], d[d * self.res <= view_range + EPS]
        m = np.maximum(np.ceil(2.0 * d - EPS).astype(int), 1)
        M = int(m.max())
        t = np.arange(1, M + 1)[None, :]
        valid = t <= m[:, None]
        frac = t / m[:, None]
        ci = np.floor(i0 + (f[:, 0:1] - i0) * frac + 0.5).astype(int).clip(0, nx - 1)
        cj = np.floor(j0 + (f[:, 1:2] - j0) * frac + 0.5).astype(int).clip(0, ny - 1)
        hit = (self.true[ci, cj] == OCC) & valid
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), M)
        mark = valid & (np.arange(M)[None, :] <= first[:, None])
        self.known[ci[mark], cj[mark]] = True
        self.known[cell] = True

    def observed(self):
        occ = np.where(self.known, self.true, UNK).astype(np.uint8)
        occupied = occ == OCC
        if occupied.any():
            clr = ndimage.distance_transform_edt(~occupied) * self.res
        else:
            clr = np.full(occ.shape, self.cap)
        return occ, np.minimum(clr, self.cap).astype(np.float32)


def box_for(shape, res=0.2):
    nx, ny = shape
    kx, ky = -(nx // 2), -(ny // 2)
    return EnuBox((kx + 0.5) * res, (kx + nx - 0.5) * res, (ky + 0.5) * res, (ky + ny - 0.5) * res, 0.2, 2.3)


def explore(true_occ, robot_cell, p, max_steps=300, done_confirm_cycles=3):
    res = p.resolution
    world = World(true_occ, res)
    box = box_for(true_occ.shape, res)
    grid = Grid.from_box(box, res, p.z_fly)
    assert (grid.nx, grid.ny) == true_occ.shape, (grid.nx, grid.ny, true_occ.shape)
    robot = tuple(robot_cell)
    world.reveal(robot, p.view_range_m)
    blacklist, goals, empty = [], [], 0
    for step in range(max_steps):
        occ, clr = world.observed()
        dec = plan(grid, occ, clr, grid.center(*robot), blacklist, p)
        if dec.stuck:
            return dict(done=False, stuck=True, goals=goals, steps=step)
        if dec.goal is None:
            empty += 1
            if empty >= done_confirm_cycles:
                return dict(done=True, goals=goals, steps=step, unexplored=len(dec.exhausted),
                            final_occ=occ, grid=grid)
            continue
        empty = 0
        goals.append((dec.goal, dec.cluster_label, dec.utility, dec.tight))
        robot = tuple(dec.goal)                        # REACHED (open-loop teleport)
        x, y = grid.center(*robot)
        blacklist.append((x, y, p.visited_radius_m))
        world.reveal(robot, p.view_range_m)
    return dict(done=False, goals=goals, steps=max_steps)
