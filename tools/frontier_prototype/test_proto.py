# filepath: tools/frontier_prototype/test_proto.py
"""Pass 1b (rev 2): test the spec's §7 claims on the literal prototype."""
import math
import time
from collections import deque
from fractions import Fraction as Fr

import numpy as np
import pytest
from hypothesis import given, settings, strategies as st, HealthCheck
from scipy import ndimage

from shim_core import EnuBox, check_goal
import proto as P

FREE, OCC, UNK = P.FREE, P.OCC, P.UNK


def blank(nx, ny, v=FREE):
    return np.full((nx, ny), v, dtype=np.uint8)


def clr_of(occ):
    o = occ == OCC
    if not o.any():
        return np.full(occ.shape, 4.0, dtype=np.float32)
    return np.minimum(ndimage.distance_transform_edt(~o) * 0.2, 4.0).astype(np.float32)


def room_with_door(door_cells):
    """Room A interior i 3..22 x j 3..14 (4.0 x 2.4 m), 2-cell east wall i 23..24 with a
    centred door, room B i 25..48 beyond. i = x (east), j = y."""
    m = blank(52, 18, OCC)
    m[3:23, 3:15] = FREE
    m[25:49, 3:15] = FREE
    j0 = 9 - door_cells // 2
    m[23:25, j0:j0 + door_cells] = FREE
    return m


# ------------------------------------------------------------ grid (§7.2)
def _exact_indices(lo, hi, res):
    return (math.ceil(Fr(str(lo)) / Fr(str(res)) - Fr(1, 2)),
            math.floor(Fr(str(hi)) / Fr(str(res)) - Fr(1, 2)))


@pytest.mark.parametrize('res', [0.05, 0.1, 0.2, 0.3])
def test_grid_indices_match_exact_decimal_arithmetic(res):
    edges = [round(v * 0.05, 2) for v in range(-100, 101)]
    for lo in edges:
        for hi in edges:
            if hi - lo < 2 * res:
                continue
            g = P.Grid.from_box(EnuBox(lo, hi, lo, hi, 0.2, 2.3), res, 1.3)
            assert (g.kx_min, g.kx_max) == _exact_indices(lo, hi, res), (lo, hi, res)


def test_grid_without_eps_has_real_failures():
    edges = [round(v * 0.05, 2) for v in range(-100, 101)]
    bad = []
    for res in (0.05, 0.1, 0.2, 0.3):
        for lo in edges:
            hi = abs(lo) + 1.0
            g = P.Grid.from_box(EnuBox(lo, hi, lo, hi, 0.2, 2.3), res, 1.3, eps=0.0)
            if (g.kx_min, g.kx_max) != _exact_indices(lo, hi, res):
                bad.append((lo, hi, res))
    print(f'\nEPS: naive float arithmetic wrong for {len(bad)} boxes, e.g. {bad[:3]}')
    assert bad


def test_grid_centre_vs_face_aligned_counts():
    centre = P.Grid.from_box(EnuBox(-4.5, 4.5, -4.5, 4.5, 0.2, 2.3), 0.2, 1.3)
    face = P.Grid.from_box(EnuBox(-1.5, 1.5, -1.5, 1.5, 0.2, 2.3), 0.1, 1.3)
    assert (centre.nx, centre.nx * centre.ny) == (46, 2116) and face.nx == 30
    assert centre.center(0, 0)[0] == pytest.approx(-4.5)
    assert face.center(0, 0)[0] == pytest.approx(-1.45)
    assert centre.z_L == pytest.approx(1.3)


def test_layer_snapping_z_fly_1_2():
    assert P.Grid.from_box(EnuBox(-1, 1, -1, 1, 0.2, 2.3), 0.2, 1.2).z_L == pytest.approx(1.3)
    assert P.Grid.from_box(EnuBox(-1, 1, -1, 1, 0.2, 2.3), 0.2, 1.2, eps=0.0).z_L == pytest.approx(1.1)


def test_world_to_cell_roundtrip():
    g = P.Grid.from_box(EnuBox(-4.5, 4.5, -4.5, 4.5, 0.2, 2.3), 0.2, 1.3)
    for i in (0, 7, 45):
        for j in (0, 23, 45):
            assert g.world_to_cell(*g.center(i, j)) == (i, j)


def test_chunks_and_validation():
    pts = list(range(10000))
    ch = P.chunks(pts, 4096)
    assert [len(c) for c in ch] == [4096, 4096, 1808] and sum(ch, []) == pts
    assert P.validate_response(3, [0, 1, 2], [0.1, 0.0, -1.0])
    assert not P.validate_response(3, [0, 1], [0.1, 0.0, -1.0])
    assert not P.validate_response(3, [0, 3, 2], [0.1, 0.0, -1.0])
    assert not P.validate_response(3, [0, 1, 2], [0.1, -0.5, -1.0])


# ------------------------------------------------------------ config C4
def test_c4_fails_on_todays_defaults():
    fence_enu = P.ned_box_to_enu(-1.5, 1.5, -1.5, 1.5, -2.5, 0.3)
    assert fence_enu == (-1.5, 1.5, -1.5, 1.5, -0.3, 2.5)
    assert not P.box_inside((-4.5, 4.5, -4.5, 4.5, 0.2, 2.3), fence_enu, 0.5)


def test_c4_asymmetric_conversion():
    assert P.ned_box_to_enu(0, 6, -2, 1, -2.5, 0.3) == (-2, 1, 0, 6, -0.3, 2.5)
    assert P.box_inside((-1.4, 0.4, 0.6, 5.4, 0.3, 1.9), P.ned_box_to_enu(0, 6, -2, 1, -2.5, 0.3), 0.5)


# ------------------------------------------------------------ frontier
def test_speckle_filtered_and_threshold():
    p = P.Params()
    occ = blank(30, 30)
    occ[5, 5] = UNK
    occ[10:12, 10:12] = UNK
    occ[20:23, 20:23] = UNK
    unk_eff = P.unknown_filter(occ, p)
    assert not unk_eff[5, 5] and not unk_eff[10, 10] and unk_eff[21, 21]
    front = P.frontier_mask(occ, unk_eff)
    assert front[19, 21] and not front[4, 5]


def test_diagonal_contact_is_not_frontier():
    p = P.Params(min_unknown_region_cells=1)
    occ = blank(10, 10, OCC)
    occ[5, 5] = FREE
    occ[6, 6] = UNK
    assert not P.frontier_mask(occ, P.unknown_filter(occ, p))[5, 5]
    occ[6, 5] = UNK
    assert P.frontier_mask(occ, P.unknown_filter(occ, p))[5, 5]


def test_grid_edge_is_not_frontier():
    occ = blank(20, 20)
    assert not P.frontier_mask(occ, P.unknown_filter(occ, P.Params())).any()


# ------------------------------------------------------------ planner
def test_dijkstra_no_corner_cutting():
    trav = np.ones((5, 5), dtype=bool)
    trav[1, 2] = False
    trav[2, 1] = False
    assert P.dijkstra(trav, (1, 1), 0.2)[2, 2] > 0.2 * math.sqrt(2) + 1e-9
    assert P.dijkstra(np.ones((3, 3), dtype=bool), (0, 0), 0.2)[1, 1] == pytest.approx(0.2 * math.sqrt(2))


def test_snap_and_stuck():
    p = P.Params()
    trav = np.zeros((10, 10), dtype=bool)
    trav[5, 8] = True
    assert P.snap(trav, (5, 5), p) == (5, 8)
    trav[5, 8] = False
    trav[5, 9] = True
    assert P.snap(trav, (5, 5), p) is None


def test_los_wall_blocks_endpoint_wall_visible():
    p = P.Params()
    blockers = np.zeros((30, 30), dtype=bool)
    blockers[15, 5:25] = True
    vis = P.visibility([(10, 10)], np.array([[20, 10], [12, 10], [15, 12]]), blockers, p)[0]
    assert list(vis) == [False, True, True]     # beyond wall hidden; near free visible; wall cell itself visible


def test_commitment():
    p = P.Params()
    assert not P.select_with_commitment(3.0, 1.5, p)
    assert P.select_with_commitment(3.6, 1.5, p)
    assert P.select_with_commitment(0.1, None, p)


# ------------------------------------------------------------ scenarios
def test_S01_doorway_1m():
    p = P.Params()
    true = room_with_door(5)
    world = P.World(true)
    world.reveal((13, 9), p.view_range_m)
    occ, clr = world.observed()
    grid = P.Grid.from_box(P.box_for(true.shape), 0.2, p.z_fly)
    dec = P.plan(grid, occ, clr, grid.center(13, 9), [], p)
    fc = np.argwhere(dec.masks['front'])
    side = 'room A' if dec.goal[0] <= 22 else ('doorway' if dec.goal[0] <= 24 else 'room B')
    print(f'\nS-01 clusters={dec.n_clusters} goal={dec.goal} side={side} tight={dec.tight} U={dec.utility:.2f} '
          f'frontier i-range=[{fc[:, 0].min()},{fc[:, 0].max()}]')
    assert dec.n_clusters == 1
    assert (fc[:, 0] >= 23).all()
    gi, gj = dec.goal
    assert dec.masks['trav'][gi, gj] and np.isfinite(dec.masks['dist'][gi, gj])


@pytest.mark.parametrize('door_cells', [2, 3])
def test_S02_narrow_door(door_cells):
    p = P.Params()
    true = room_with_door(door_cells)
    door_clr = clr_of(true)[23:25, 9 - door_cells // 2: 9 - door_cells // 2 + door_cells].max()
    res = P.explore(true, (13, 9), p)
    gi = [g[0][0] for g in res['goals']]
    print(f'\nS-02 door {door_cells * 0.2:.1f} m: door clr {door_clr:.2f} m grid-traversable={door_clr >= 0.4} '
          f'done={res["done"]} goals={len(gi)} tight={sum(g[3] for g in res["goals"])} '
          f'max goal i={max(gi)} unexplored={res.get("unexplored")}')
    assert res['done']
    if door_clr < p.traverse_clearance_m:
        assert all(i <= 22 for i in gi) and res['unexplored'] >= 1
    else:
        assert res['unexplored'] == 0 and max(gi) >= 25


def test_S03_corridor_dead_end():
    p = P.Params()
    m = blank(70, 20, OCC)
    m[3:23, 3:17] = FREE
    m[23:65, 7:13] = FREE                   # corridor 8.4 x 1.2 m
    res = P.explore(m, (10, 10), p)
    print(f'\nS-03 done={res["done"]} goals={len(res["goals"])} {[g[0] for g in res["goals"]]} unexplored={res.get("unexplored")}')
    assert res['done'] and res['unexplored'] == 0
    assert max(g[0][0] for g in res['goals']) > 40
    assert not (res['final_occ'] == UNK)[m == FREE].any()


def test_S04_enclosed_known_room():
    m = blank(24, 18, OCC)
    m[3:21, 3:15] = FREE
    res = P.explore(m, (12, 9), P.Params())
    assert res['done'] and res['goals'] == [] and res['steps'] == 2


def test_S05_speckle():
    p = P.Params()
    rng = np.random.default_rng(3)
    occ = blank(40, 40)
    occ[0:2, :] = OCC
    occ[-2:, :] = OCC
    occ[:, 0:2] = OCC
    occ[:, -2:] = OCC
    for _ in range(25):
        i, j = rng.integers(4, 35, size=2)
        h, w = rng.integers(1, 3, size=2)
        occ[i:i + h, j:j + w] = UNK
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, p.z_fly)
    dec = P.plan(grid, occ, clr_of(occ), grid.center(20, 20), [], p)
    assert dec.n_clusters == 0 and dec.goal is None


def two_rooms_corridors(w):
    m = blank(90, 22, OCC)
    j0 = 11 - w // 2
    m[3:12, 5:17] = FREE          # room W
    m[12:22, j0:j0 + w] = FREE    # west corridor 2.0 m
    m[22:36, 4:18] = FREE         # start room
    m[36:61, j0:j0 + w] = FREE    # east corridor 5.0 m
    m[61:86, 3:19] = FREE         # room E
    return m


@pytest.mark.parametrize('w', [4, 6])
def test_S06_rooms_via_corridors(w):
    p = P.Params()
    m = two_rooms_corridors(w)
    res = P.explore(m, (25, 11), p)
    goals = [g[0] for g in res['goals']]
    print(f'\nS-06 corridor {w * 0.2:.1f} m: done={res["done"]} goals={len(goals)} tight={sum(g[3] for g in res["goals"])} '
          f'first={goals[0] if goals else None} unexplored={res.get("unexplored")}')
    assert res['done'] and res['unexplored'] == 0
    assert goals[0][0] < 25, 'nearer (west) frontier first'
    assert not (res['final_occ'] == UNK)[m == FREE].any()


def test_S06b_without_tight_goals_corridor_08_unexplored():
    p = P.Params(allow_tight_goals=False)
    res = P.explore(two_rooms_corridors(4), (25, 11), p)
    print(f'\nS-06b tight disabled, 0.8 m corridors: done={res["done"]} unexplored={res.get("unexplored")}')
    assert res['done'] and res['unexplored'] >= 1


def test_S07_symmetric_exact_tie():
    p = P.Params()
    m = blank(61, 18, OCC)
    m[20:41, 3:15] = FREE
    m[3:20, 7:11] = FREE
    m[41:58, 7:11] = FREE
    world = P.World(m)
    world.reveal((30, 9), p.view_range_m)
    occ, clr = world.observed()
    occ[31:, :] = occ[29::-1, :][:30]     # exact mirror about i = 30
    clr[31:, :] = clr[29::-1, :][:30]
    grid = P.Grid.from_box(P.box_for(m.shape), 0.2, p.z_fly)
    d1 = P.plan(grid, occ, clr, grid.center(30, 9), [], p)
    d2 = P.plan(grid, occ.copy(), clr.copy(), grid.center(30, 9), [], p)
    Us = {L: round(v[2], 12) for L, v in d1.per_cluster.items()}
    print(f'\nS-07 goal={d1.goal} label={d1.cluster_label} utilities={Us} sizes={ {L: v[4] for L, v in d1.per_cluster.items()} }')
    assert (d1.goal, d1.utility) == (d2.goal, d2.utility)
    vals = sorted(d1.per_cluster.items(), key=lambda kv: kv[0])
    if len(vals) == 2 and abs(vals[0][1][2] - vals[1][1][2]) < 1e-12 and vals[0][1][4] == vals[1][1][4]:
        assert d1.cluster_label == vals[0][0] and d1.goal[0] < 30, 'exact tie -> lower label (west)'
    else:
        pytest.fail(f'fixture did not produce an exact tie: {Us}')


def test_S08_unknown_only_outside_box():
    occ = blank(30, 30)
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, 1.3)
    assert P.plan(grid, occ, clr_of(occ), grid.center(15, 15), [], P.Params()).goal is None


def test_S09_commitment_against_current_goal():
    p = P.Params()
    true = room_with_door(5)
    world = P.World(true)
    world.reveal((13, 9), p.view_range_m)
    occ, clr = world.observed()
    grid = P.Grid.from_box(P.box_for(true.shape), 0.2, p.z_fly)
    dec = P.plan(grid, occ, clr, grid.center(13, 9), [], p)
    U_cur = P.utility_of_goal(dec.goal, grid, occ, clr, dec, [], p)
    assert U_cur == pytest.approx(dec.utility)
    assert not P.select_with_commitment(dec.utility, U_cur, p)


# ------------------------------------------------------------ properties
def brute_d_unk(unk_eff, i, j, res):
    cells = np.argwhere(unk_eff)
    return math.inf if len(cells) == 0 else float(np.min(np.hypot(cells[:, 0] - i, cells[:, 1] - j))) * res


def bfs_reachable(trav, start):
    nx, ny = trav.shape
    seen = np.zeros_like(trav)
    q = deque([start])
    seen[start] = True
    while q:
        i, j = q.popleft()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                ni, nj = i + di, j + dj
                if (di or dj) and 0 <= ni < nx and 0 <= nj < ny and trav[ni, nj] and not seen[ni, nj]:
                    if di and dj and not (trav[i + di, j] and trav[i, j + dj]):
                        continue
                    seen[ni, nj] = True
                    q.append((ni, nj))
    return seen


@st.composite
def random_map(draw):
    nx = draw(st.integers(20, 40))
    ny = draw(st.integers(20, 40))
    rng = np.random.default_rng(draw(st.integers(0, 2**31 - 1)))
    occ = np.full((nx, ny), FREE, dtype=np.uint8)
    for _ in range(rng.integers(0, 6)):
        i, j = rng.integers(0, [nx, ny])
        h, w = rng.integers(1, 8, size=2)
        occ[i:i + h, j:j + w] = OCC
    for _ in range(rng.integers(0, 4)):
        i, j = rng.integers(0, [nx, ny])
        h, w = rng.integers(1, 12, size=2)
        occ[i:i + h, j:j + w] = UNK
    free = np.argwhere(occ == FREE)
    if len(free) == 0:
        occ[nx // 2, ny // 2] = FREE
        free = np.argwhere(occ == FREE)
    r = tuple(free[rng.integers(0, len(free))])
    bl = [(float(rng.uniform(-3, 3)), float(rng.uniform(-3, 3)), float(rng.uniform(0.3, 1.2)))
          for _ in range(rng.integers(0, 4))]
    return occ, r, bl


@settings(max_examples=200, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(random_map())
def test_P01_P04_goal_admissibility(sample):
    occ, r, bl = sample
    p = P.Params()
    clr = clr_of(occ)
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, p.z_fly)
    dec = P.plan(grid, occ, clr, grid.center(*r), bl, p)
    if dec.goal is None:
        return
    i, j = dec.goal
    x, y = grid.center(i, j)
    ok, why = check_goal((x, y, grid.z_L), grid.box, p.obstacle_inflation)
    assert ok, why
    unk_eff = P.unknown_filter(occ, p)
    du = brute_d_unk(unk_eff, i, j, 0.2)
    assert occ[i, j] == FREE and clr[i, j] >= p.traverse_clearance_m and du >= p.traverse_clearance_m - 1e-9
    if not dec.tight:
        assert clr[i, j] >= p.goal_clearance_m and du >= p.unknown_standoff_m - 1e-9
    trav = (occ == FREE) & (clr >= p.traverse_clearance_m)
    start = P.snap(trav, grid.world_to_cell(*grid.center(*r)), p)
    assert bfs_reachable(trav, start)[i, j]
    assert not any(math.hypot(x - bx, y - by) <= rad for bx, by, rad in bl)


@settings(max_examples=80, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(random_map())
def test_P02_no_unknown_no_goal(sample):
    occ, r, bl = sample
    occ = occ.copy()
    occ[occ == UNK] = FREE
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, 1.3)
    assert P.plan(grid, occ, clr_of(occ), grid.center(*r), bl, P.Params()).goal is None


@settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(random_map())
def test_P03_determinism(sample):
    occ, r, bl = sample
    p = P.Params()
    clr = clr_of(occ)
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, p.z_fly)
    a = P.plan(grid, occ.copy(), clr.copy(), grid.center(*r), list(bl), p)
    b = P.plan(grid, occ.copy(), clr.copy(), grid.center(*r), list(bl), p)
    assert (a.goal, a.utility, a.cluster_label, a.n_clusters, a.tight) == (b.goal, b.utility, b.cluster_label, b.n_clusters, b.tight)


# ------------------------------------------------------------ B-02
def lattice_map(n, k):
    occ = blank(n, n)
    occ[0:2, :] = OCC
    occ[-2:, :] = OCC
    occ[:, 0:2] = OCC
    occ[:, -2:] = OCC
    side = math.ceil(math.sqrt(k))
    step = n // (side + 1)
    placed = 0
    for a in range(1, side + 1):
        for b in range(1, side + 1):
            if placed == k:
                break
            ci, cj = a * step, b * step
            occ[ci - 2:ci + 3, cj - 2:cj + 3] = UNK
            placed += 1
    return occ


@pytest.mark.parametrize('n', [46, 100, 200])
@pytest.mark.parametrize('k', [1, 8, 32])
def test_B02_planning_time(n, k):
    p = P.Params()
    occ = lattice_map(n, k)
    clr = clr_of(occ)
    grid = P.Grid.from_box(P.box_for(occ.shape), 0.2, p.z_fly)
    free = np.argwhere(occ == FREE)
    r = tuple(free[np.argmin(np.hypot(free[:, 0] - n // 2 - 1, free[:, 1] - n // 2 - 1))])
    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        dec = P.plan(grid, occ, clr, grid.center(*r), [], p)
        times.append((time.perf_counter() - t0) * 1e3)
    print(f'\nB-02 N={n * n:6d} k={k:2d} clusters={dec.n_clusters:2d} plan median {sorted(times)[1]:8.1f} ms')
