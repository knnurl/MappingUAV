# filepath: src/sim_bringup/test/test_world.py
import math

import numpy as np

from sim_bringup.world import WORLDS, ray_directions, scan, two_rooms


def test_fov_matches_the_mid360():
    d = ray_directions()
    el = np.degrees(np.arcsin(d[:, 2]))
    assert el.min() >= -7.01 and el.max() <= 52.01
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)


def test_scan_hits_stay_on_surfaces_and_respect_the_blind_range():
    w = two_rooms()
    pts = scan(w, (0.0, 0.0, 1.3), 0.0, ray_directions())
    assert len(pts) > 1000
    r = np.linalg.norm(pts - np.array([0.0, 0.0, 1.3]), axis=1)
    assert r.min() >= 0.5
    on_ceiling = pts[:, 2] >= w.ceiling_z - 0.06
    on_floor = pts[:, 2] <= w.floor_z + 0.06
    on_wall = w.occupied(pts[:, 0], pts[:, 1])
    assert (on_ceiling | on_floor | on_wall).all()


def test_door_lets_rays_into_the_second_room_and_the_wall_blocks_them():
    w = two_rooms()
    pts = scan(w, (0.0, 0.0, 1.3), 0.0, ray_directions())
    beyond = pts[pts[:, 0] > 3.6]
    assert len(beyond) > 0 and np.abs(beyond[:, 1]).max() < 2.1
    assert not w.hits_wall(3.2, 0.0) and w.hits_wall(3.2, 1.5)


def test_scan_rotates_with_yaw():
    w = two_rooms()
    a = scan(w, (0.0, 0.0, 1.3), 0.0, ray_directions(az_step_deg=90, el_count=1,
                                                      el_min_deg=0, el_max_deg=0))
    b = scan(w, (0.0, 0.0, 1.3), math.pi / 2, ray_directions(az_step_deg=90, el_count=1,
                                                              el_min_deg=0, el_max_deg=0))
    assert np.allclose(sorted(map(tuple, np.round(a, 2))), sorted(map(tuple, np.round(b, 2))))


def test_every_world_starts_in_free_space():
    for make in WORLDS.values():
        assert not make().hits_wall(0.0, 0.0)
