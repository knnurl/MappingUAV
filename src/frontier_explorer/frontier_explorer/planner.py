# filepath: src/frontier_explorer/frontier_explorer/planner.py
"""Traversability, reachability, viewpoints, gain, utility and commitment
(spec §7.8, §7.10 - §7.14). No ROS imports. Ported from the verified prototype.
"""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from planner_shim.shim_core import check_goal

from .frontier import (
    clusters, dist_to_unknown, frontier_mask, unknown_filter,
)
from .grid import EPS, FREE, OCC

def traversable(occ, clr, p):
    return (occ == FREE) & (clr >= p.traverse_clearance_m) & (clr >= 0)


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
