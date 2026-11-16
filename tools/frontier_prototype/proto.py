# filepath: tools/frontier_prototype/proto.py
"""Literal prototype of spec §7 (frontier_explorer core). VERIFICATION ONLY.

Revision 2: implements the pass-1 corrections (LOS endpoint exclusion, tiered
candidates, cluster-proximity tie-break, U_cur definition, vectorised §7.12).
"""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from shim_core import EnuBox, check_goal

FREE, OCC, UNK = 0, 1, 2
EPS = 1e-6
S4 = ndimage.generate_binary_structure(2, 1)
S8 = ndimage.generate_binary_structure(2, 2)


@dataclass
class Params:
    resolution: float = 0.2
    z_fly: float = 1.3
    obstacle_inflation: float = 0.4
    min_unknown_region_cells: int = 9
    min_cluster_cells: int = 3
    max_clusters: int = 32
    traverse_clearance_m: float = 0.4
    goal_clearance_m: float = 0.6
    unknown_standoff_m: float = 0.6
    allow_tight_goals: bool = True
    view_range_m: float = 3.0
    candidate_stride_cells: int = 2
    max_candidates_per_cluster: int = 256
    max_gain_samples: int = 64
    start_snap_radius_m: float = 0.6
    distance_weight: float = 1.0
    switch_margin: float = 2.0
    visited_radius_m: float = 0.6
    blacklist_radius_m: float = 1.0


# ---------------------------------------------------------------- §7.2 grid
@dataclass
class Grid:
    box: EnuBox
    res: float
    kx_min: int
    kx_max: int
    ky_min: int
    ky_max: int
    k_z: int

    @staticmethod
    def from_box(box, res, z_fly, eps=EPS):
        return Grid(box, res,
                    math.ceil(box.x_min / res - 0.5 - eps), math.floor(box.x_max / res - 0.5 + eps),
                    math.ceil(box.y_min / res - 0.5 - eps), math.floor(box.y_max / res - 0.5 + eps),
                    math.floor(z_fly / res + eps))

    @property
    def nx(self):
        return self.kx_max - self.kx_min + 1

    @property
    def ny(self):
        return self.ky_max - self.ky_min + 1

    @property
    def z_L(self):
        return (self.k_z + 0.5) * self.res

    def center(self, i, j):
        return ((self.kx_min + i + 0.5) * self.res, (self.ky_min + j + 0.5) * self.res)

    def world_to_cell(self, x, y):
        i = math.floor(x / self.res + EPS) - self.kx_min
        j = math.floor(y / self.res + EPS) - self.ky_min
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def query_points(self):
        return [(*self.center(i, j), self.z_L) for i in range(self.nx) for j in range(self.ny)]


def chunks(points, n):
    return [points[k:k + n] for k in range(0, len(points), n)]


def validate_response(n_points, occupancy, clearance):
    if len(occupancy) != n_points or len(clearance) != n_points:
        return False
    if any(o not in (0, 1, 2) for o in occupancy):
        return False
    if any((c < 0 and c != -1.0) for c in clearance):
        return False
    return True


# ------------------------------------------------------------ §7.5 - §7.9
def unknown_filter(occ, p):
    unk = occ == UNK
    lab, n = ndimage.label(unk, S4)
    if n == 0:
        return np.zeros_like(unk)
    sizes = ndimage.sum(unk, lab, index=np.arange(1, n + 1))
    keep = np.zeros(n + 1, dtype=bool)
    keep[1:] = sizes >= p.min_unknown_region_cells
    return keep[lab]


def frontier_mask(occ, unk_eff):
    return (occ == FREE) & ndimage.binary_dilation(unk_eff, S4, border_value=0)


def clusters(front, p):
    lab, n = ndimage.label(front, S8)
    out = []
    for L in range(1, n + 1):
        cells = np.argwhere(lab == L)            # raster order over [i, j]
        if len(cells) >= p.min_cluster_cells:
            out.append((L, cells))
    if len(out) > p.max_clusters:
        out = sorted(out, key=lambda c: (-len(c[1]), c[0]))[:p.max_clusters]
        out.sort(key=lambda c: c[0])
    return out


def traversable(occ, clr, p):
    return (occ == FREE) & (clr >= p.traverse_clearance_m) & (clr >= 0)


def dist_to_unknown(unk_eff, res):
    if not unk_eff.any():
        return np.full(unk_eff.shape, np.inf)
    return ndimage.distance_transform_edt(~unk_eff) * res


# ------------------------------------------------------------ §7.10 - §7.11
def snap(trav, s0, p):
    i0, j0 = s0
    if trav[i0, j0]:
        return s0
    r = int(math.floor(p.start_snap_radius_m / p.resolution + EPS))
    best = None
    for i in range(max(0, i0 - r), min(trav.shape[0], i0 + r + 1)):
        for j in range(max(0, j0 - r), min(trav.shape[1], j0 + r + 1)):
            if not trav[i, j]:
                continue
            d = math.hypot(i - i0, j - j0) * p.resolution
            if d <= p.start_snap_radius_m + EPS:
                key = (d, i, j)
                if best is None or key < best:
                    best = key
    return None if best is None else (best[1], best[2])


def dijkstra(trav, start, res):
    nx, ny = trav.shape
    dist = np.full(trav.shape, np.inf)
    dist[start] = 0.0
    heap = [(0.0, start[0], start[1])]
    diag = res * math.sqrt(2.0)
    while heap:
        d, i, j = heapq.heappop(heap)
        if d > dist[i, j]:
            continue
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny) or not trav[ni, nj]:
                    continue
                if di and dj and not (trav[i + di, j] and trav[i, j + dj]):
                    continue                     # no corner cutting
                nd = d + (diag if di and dj else res)
                if nd < dist[ni, nj]:
                    dist[ni, nj] = nd
                    heapq.heappush(heap, (nd, ni, nj))
    return dist


# ------------------------------------------------------------------ §7.12
def visibility(K, S, blockers, p):
    """K: (nk, 2) sources, S: (ns, 2) targets (cell indices).
    vis[k, s] = in view range AND line of sight. Samples every res/2 between cell
    centres, endpoints excluded by index; nearest cell = floor(p + 0.5); a sample
    whose nearest cell IS the source or target cell never blocks."""
    K = np.asarray(K, dtype=float).reshape(-1, 2)
    S = np.asarray(S, dtype=float).reshape(-1, 2)
    di = S[None, :, 0] - K[:, None, 0]
    dj = S[None, :, 1] - K[:, None, 1]
    d = np.hypot(di, dj)                                   # cells
    inr = d * p.resolution <= p.view_range_m + EPS
    m = np.where(inr, np.ceil(2.0 * d - EPS), 0).astype(int)
    vis = inr.copy()
    M = int(m.max()) if m.size else 0
    if M >= 2:
        t = np.arange(1, M)[None, None, :]
        valid = t < m[:, :, None]
        frac = t / np.maximum(m[:, :, None], 1)
        ci = np.floor(K[:, None, None, 0] + di[:, :, None] * frac + 0.5).astype(int)
        cj = np.floor(K[:, None, None, 1] + dj[:, :, None] * frac + 0.5).astype(int)
        ci = ci.clip(0, blockers.shape[0] - 1)
        cj = cj.clip(0, blockers.shape[1] - 1)
        ki = K[:, 0].astype(int)[:, None, None]
        kj = K[:, 1].astype(int)[:, None, None]
        si = S[:, 0].astype(int)[None, :, None]
        sj = S[:, 1].astype(int)[None, :, None]
        is_end = ((ci == ki) & (cj == kj)) | ((ci == si) & (cj == sj))
        blocked = (blockers[ci, cj] & valid & ~is_end).any(axis=2)
        vis &= ~blocked
    return vis


def cluster_candidates(C, grid, occ, clr, trav, d_unk, dist, blacklist, p, cap=True):
    """Returns (cands (n,2) int, tier, d_cluster per cand)."""
    nx, ny = occ.shape
    rc = int(math.ceil(p.view_range_m / p.resolution - EPS))
    i0, j0 = int(max(C[:, 0].min() - rc, 0)), int(max(C[:, 1].min() - rc, 0))
    i1, j1 = int(min(C[:, 0].max() + rc, nx - 1)), int(min(C[:, 1].max() + rc, ny - 1))
    s = p.candidate_stride_cells
    ii = np.arange(i0 + (-i0) % s, i1 + 1, s)
    jj = np.arange(j0 + (-j0) % s, j1 + 1, s)
    empty = (np.zeros((0, 2), dtype=int), None, np.zeros(0))
    if len(ii) == 0 or len(jj) == 0:
        return empty
    I, J = (a.ravel() for a in np.meshgrid(ii, jj, indexing='ij'))
    cmask = np.zeros((i1 - i0 + 1, j1 - j0 + 1), dtype=bool)
    cmask[C[:, 0] - i0, C[:, 1] - j0] = True
    dcl = (ndimage.distance_transform_edt(~cmask) * p.resolution)[I - i0, J - j0]
    base = trav[I, J] & np.isfinite(dist[I, J]) & (dcl <= p.view_range_m + EPS)
    X = (grid.kx_min + I + 0.5) * p.resolution
    Y = (grid.ky_min + J + 0.5) * p.resolution
    if blacklist:
        bl = np.asarray(blacklist, dtype=float)
        base &= ~(np.hypot(X[:, None] - bl[None, :, 0], Y[:, None] - bl[None, :, 1]) <= bl[None, :, 2]).any(axis=1)
    tiers = [('preferred', base & (clr[I, J] >= p.goal_clearance_m) & (d_unk[I, J] >= p.unknown_standoff_m))]
    if p.allow_tight_goals:
        tiers.append(('tight', base & (d_unk[I, J] >= p.traverse_clearance_m)))
    for tier, mask in tiers:
        idx = [k for k in np.nonzero(mask)[0]
               if check_goal((float(X[k]), float(Y[k]), grid.z_L), grid.box, p.obstacle_inflation)[0]]
        if not idx:
            continue
        idx = np.asarray(idx)
        if cap and len(idx) > p.max_candidates_per_cluster:
            order = np.lexsort((J[idx], I[idx], dist[I[idx], J[idx]]))
            idx = idx[order[:p.max_candidates_per_cluster]]
        return np.stack([I[idx], J[idx]], axis=1), tier, dcl[idx]
    return empty


def gain_samples(C, p):
    if len(C) <= p.max_gain_samples:
        return C
    step = math.ceil(len(C) / p.max_gain_samples)
    return C[::step]


@dataclass
class Decision:
    goal: tuple = None
    utility: float = -math.inf
    cluster_label: int = None
    tight: bool = False
    visible: np.ndarray = None
    n_clusters: int = 0
    exhausted: list = field(default_factory=list)
    per_cluster: dict = field(default_factory=dict)
    stuck: bool = False
    masks: dict = field(default_factory=dict)


def best_viewpoint(C, K, dcl, dist, blockers, p):
    S = gain_samples(C, p)
    vis = visibility(K, S, blockers, p)
    gain = vis.sum(axis=1) * len(C) / len(S)
    # §7.12 step 6 (rev 2): max gain, then closest to cluster, then path distance, then (i, j)
    order = np.lexsort((K[:, 1], K[:, 0], dist[K[:, 0], K[:, 1]], dcl, -gain))
    b = order[0]
    return tuple(int(v) for v in K[b]), float(gain[b]), S[vis[b]]


def plan(grid, occ, clr, pose_xy, blacklist, p):
    unk_eff = unknown_filter(occ, p)
    front = frontier_mask(occ, unk_eff)
    cls = clusters(front, p)
    trav = traversable(occ, clr, p)
    d_unk = dist_to_unknown(unk_eff, p.resolution)
    blockers = (occ == OCC) | unk_eff
    dec = Decision(n_clusters=len(cls))
    dec.masks = dict(unk_eff=unk_eff, front=front, trav=trav, d_unk=d_unk, blockers=blockers, clusters=cls)
    start = snap(trav, grid.world_to_cell(*pose_xy), p)
    if start is None:
        dec.stuck = True
        return dec
    dist = dijkstra(trav, start, p.resolution)
    dec.masks['dist'] = dist
    best_key = None
    for L, C in cls:
        K, tier, dcl = cluster_candidates(C, grid, occ, clr, trav, d_unk, dist, blacklist, p)
        if tier is None:
            dec.exhausted.append((L, C))
            continue
        k, g, vis = best_viewpoint(C, K, dcl, dist, blockers, p)
        if g == 0:
            dec.exhausted.append((L, C))
            continue
        U = g * p.resolution - p.distance_weight * dist[k]
        dec.per_cluster[L] = (k, g, U, vis, len(C), tier)
        sel = (-U, -len(C), L)                      # §7.14 ties: larger |C|, lower label
        if best_key is None or sel < best_key:
            best_key = sel
            dec.goal, dec.utility, dec.cluster_label, dec.visible = k, U, L, vis
            dec.tight = tier == 'tight'
    return dec


def utility_of_goal(g, grid, occ, clr, dec, blacklist, p):
    """§7.14 (rev 2) U_cur = max over kept clusters C for which g is a candidate (uncapped,
    same tier rules) of gain_C(g)·res − w·dist[g]; None if g is a candidate of no cluster."""
    m = dec.masks
    best = None
    for L, C in m['clusters']:
        K, tier, dcl = cluster_candidates(C, grid, occ, clr, m['trav'], m['d_unk'], m['dist'], blacklist, p, cap=False)
        if tier is None or not (K == np.asarray(g)).all(axis=1).any():
            continue
        S = gain_samples(C, p)
        gval = visibility([g], S, m['blockers'], p).sum() * len(C) / len(S)
        U = gval * p.resolution - p.distance_weight * m['dist'][tuple(g)]
        best = U if best is None else max(best, U)
    return best


def select_with_commitment(U_best, U_cur, p):
    if U_cur is None:
        return True
    return U_best > U_cur + p.switch_margin


# ------------------------------------------------------- config (§6, C4)
def ned_box_to_enu(x_min, x_max, y_min, y_max, z_min, z_max):
    return (y_min, y_max, x_min, x_max, -z_max, -z_min)


def box_inside(inner, outer, shrink):
    ix0, ix1, iy0, iy1, iz0, iz1 = inner
    ox0, ox1, oy0, oy1, oz0, oz1 = outer
    return (ix0 >= ox0 + shrink and ix1 <= ox1 - shrink and iy0 >= oy0 + shrink and iy1 <= oy1 - shrink
            and iz0 >= oz0 + shrink and iz1 <= oz1 - shrink)


# ------------------------------------------- open-loop reveal script (§13.2)
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
