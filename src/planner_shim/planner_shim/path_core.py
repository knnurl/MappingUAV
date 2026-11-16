# filepath: src/planner_shim/planner_shim/path_core.py
"""Pure path-planning logic for go_to_with_avoidance (WP-D, design B, operator
decision D2 of 2026-11-16): a grid path on one horizontal slice of the
map_interface occupancy map, executed by Aerostack2's FollowPath behaviour.

No ROS imports. Frames: odom / ENU throughout.

Safety rules encoded here:
- A cell is traversable only if it was OBSERVED free and its 3D clearance is at
  least the inflation. Unknown space is never traversable: the map server's
  distance field treats unknown as free, so occupancy is checked separately.
- Diagonal steps never cut a corner.
- A vertical move is allowed only through observed-free cells (the Mid-360
  does not see straight down, so a descent into unseen space is refused).

Limits: this is a static-scene planner. Nothing here reacts faster than the
map (2 Hz insertion) and the 1 Hz path re-check in the node.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

FREE, OCC, UNK = 0, 1, 2
EPS = 1e-6


@dataclass(frozen=True)
class SliceGrid:
    """Cells are map voxels: cell (i, j) is centred on a voxel centre, so a
    query at the cell centre never falls on a voxel boundary."""
    res: float
    kx_min: int
    kx_max: int
    ky_min: int
    ky_max: int

    @staticmethod
    def from_box(box, res):
        if res <= 0:
            raise ValueError('resolution must be > 0')
        g = SliceGrid(
            res,
            math.ceil(box.x_min / res - 0.5 - EPS), math.floor(box.x_max / res - 0.5 + EPS),
            math.ceil(box.y_min / res - 0.5 - EPS), math.floor(box.y_max / res - 0.5 + EPS))
        if g.nx < 1 or g.ny < 1:
            raise ValueError('planning box holds no cell at this resolution')
        return g

    @property
    def nx(self):
        return self.kx_max - self.kx_min + 1

    @property
    def ny(self):
        return self.ky_max - self.ky_min + 1

    def center(self, i, j):
        return ((self.kx_min + i + 0.5) * self.res, (self.ky_min + j + 0.5) * self.res)

    def cell(self, x, y):
        """World -> cell, or None outside the grid."""
        i = math.floor(x / self.res + EPS) - self.kx_min
        j = math.floor(y / self.res + EPS) - self.ky_min
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return i, j
        return None

    def layer_center(self, z):
        return (math.floor(z / self.res + EPS) + 0.5) * self.res

    def slice_points(self, z):
        zc = self.layer_center(z)
        return [(*self.center(i, j), zc) for i in range(self.nx) for j in range(self.ny)]

    def column_points(self, x, y, z_from, z_to):
        """Voxel centres of the column at (x, y) strictly between the layers of
        z_from and z_to, plus both end layers."""
        k0 = math.floor(min(z_from, z_to) / self.res + EPS)
        k1 = math.floor(max(z_from, z_to) / self.res + EPS)
        c = self.cell(x, y)
        if c is None:
            return []
        cx, cy = self.center(*c)
        return [(cx, cy, (k + 0.5) * self.res) for k in range(k0, k1 + 1)]


def traversable(occ, clr, clearance_m):
    """occ, clr: arrays of shape (nx, ny). clr < 0 means 'no answer'."""
    occ = np.asarray(occ)
    clr = np.asarray(clr)
    return (occ == FREE) & (clr >= clearance_m) & (clr >= 0)


def points_free(occ, clr, clearance_m):
    """True if every queried point is observed free with clearance."""
    occ = np.asarray(occ)
    clr = np.asarray(clr)
    return bool(len(occ) > 0 and ((occ == FREE) & (clr >= clearance_m)).all())


def snap(trav, cell, radius_cells):
    """Nearest traversable cell within radius_cells of cell (ties: lowest
    (i, j)); the cell itself if traversable; None if there is none."""
    i0, j0 = cell
    if trav[i0, j0]:
        return cell
    r = int(math.floor(radius_cells + EPS))
    best = None
    for i in range(max(0, i0 - r), min(trav.shape[0], i0 + r + 1)):
        for j in range(max(0, j0 - r), min(trav.shape[1], j0 + r + 1)):
            if trav[i, j]:
                d = math.hypot(i - i0, j - j0)
                if d <= radius_cells + EPS and (best is None or (d, i, j) < best):
                    best = (d, i, j)
    return None if best is None else (best[1], best[2])


def astar(trav, start, goal):
    """Shortest 8-connected path of cells from start to goal over trav, without
    corner cutting. Returns [start, ..., goal] or None. Deterministic."""
    nx, ny = trav.shape
    if not (trav[start] and trav[goal]):
        return None
    if start == goal:
        return [start]
    diag = math.sqrt(2.0)

    def h(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (diag - 2.0) * min(dx, dy)

    best = {start: 0.0}
    parent = {}
    heap = [(h(start), 0.0, start[0], start[1])]
    while heap:
        _, g, i, j = heapq.heappop(heap)
        if (i, j) == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        if g > best.get((i, j), math.inf):
            continue
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny) or not trav[ni, nj]:
                    continue
                if di and dj and not (trav[i + di, j] and trav[i, j + dj]):
                    continue
                ng = g + (diag if di and dj else 1.0)
                if ng < best.get((ni, nj), math.inf) - 1e-12:
                    best[(ni, nj)] = ng
                    parent[(ni, nj)] = (i, j)
                    heapq.heappush(heap, (ng + h((ni, nj)), ng, ni, nj))
    return None


def line_clear(trav, a, b):
    """True if the straight segment between cell coordinates a and b (floats,
    in cells) stays on traversable cells. Sampled every quarter cell; both
    cells adjoining a sample that sits on a cell edge must be traversable."""
    (ai, aj), (bi, bj) = a, b
    n = max(1, int(math.ceil(4.0 * math.hypot(bi - ai, bj - aj))))
    nx, ny = trav.shape
    for t in range(n + 1):
        x = ai + (bi - ai) * t / n
        y = aj + (bj - aj) * t / n
        for ci in {int(math.floor(x + 0.5 - 1e-9)), int(math.floor(x + 0.5 + 1e-9))}:
            for cj in {int(math.floor(y + 0.5 - 1e-9)), int(math.floor(y + 0.5 + 1e-9))}:
                if not (0 <= ci < nx and 0 <= cj < ny) or not trav[ci, cj]:
                    return False
    return True


def shortcut(path, trav):
    """Greedy line-of-sight pruning of a cell path. Keeps both ends."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_clear(trav, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_path(grid, occ, clr, start_xy, goal_xy, clearance_m, snap_radius_m):
    """Returns (waypoints_xy, reason). waypoints_xy is a list of (x, y) from the
    first cell AFTER the start to the goal cell centre, or None with a reason."""
    trav = traversable(occ, clr, clearance_m)
    s = grid.cell(*start_xy)
    g = grid.cell(*goal_xy)
    if s is None:
        return None, 'vehicle is outside the planning box'
    if g is None:
        return None, 'goal is outside the planning box'
    s = snap(trav, s, snap_radius_m / grid.res)
    if s is None:
        return None, 'vehicle is not in traversable space'
    if not trav[g]:
        return None, 'goal cell is not observed free with clearance'
    cells = astar(trav, s, g)
    if cells is None:
        return None, 'no path through observed free space'
    cells = shortcut(cells, trav)
    pts = [grid.center(*c) for c in cells]
    return (pts[1:] if len(pts) > 1 else pts), ''


def path_clear(grid, occ, clr, robot_xy, waypoints_xy, clearance_m, ignore_radius_m):
    """Re-check of the path still to be flown: from the robot through the
    remaining waypoints. Samples within ignore_radius_m of the robot are not
    judged (the vehicle's own cell can read as low clearance)."""
    trav = traversable(occ, clr, clearance_m)
    pts = [robot_xy] + list(waypoints_xy)
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.ceil(4.0 * math.hypot(bx - ax, by - ay) / grid.res)))
        for t in range(n + 1):
            x = ax + (bx - ax) * t / n
            y = ay + (by - ay) * t / n
            if math.hypot(x - robot_xy[0], y - robot_xy[1]) <= ignore_radius_m:
                continue
            c = grid.cell(x, y)
            if c is None or not trav[c]:
                return False
    return True


def remaining_waypoints(waypoints_xy, robot_xy, reach_m):
    """Drop leading waypoints the vehicle has already come within reach_m of."""
    k = 0
    for idx, (x, y) in enumerate(waypoints_xy[:-1]):
        if math.hypot(x - robot_xy[0], y - robot_xy[1]) <= reach_m:
            k = idx + 1
    return list(waypoints_xy[k:])


def stop_reason(now, *, pose_t, pose_timeout_s, lio_state, lio_t, lio_timeout_s,
                platform_t, platform_timeout_s, offboard, flying):
    """Why motion must not start or must stop; '' if nothing forbids it.

    lio_state: 0 OK, 1 DEGRADED, None never received. Times are seconds on the
    caller's clock, None when never received. Silence counts as a fault: a
    health monitor that has died cannot vouch for anything.
    """
    if pose_t is None or now - pose_t > pose_timeout_s:
        return 'no fresh vehicle pose'
    if lio_t is None or now - lio_t > lio_timeout_s:
        return 'LIO health unknown (no /lio_health/state)'
    if lio_state != 0:
        return 'LIO degraded'
    if platform_t is None or now - platform_t > platform_timeout_s:
        return 'no fresh platform info'
    if not offboard:
        return ('platform is not in Offboard (after any Offboard exit the '
                'platform node must be restarted)')
    if not flying:
        return 'platform is not FLYING'
    return ''
