# filepath: src/sim_bringup/sim_bringup/world.py
"""Box worlds and a Mid-360-like raycaster for the closed-loop rig. No ROS.

A world is a 2D grid of walls extruded from the floor to the ceiling, in the
FAST-LIO frame: origin where the vehicle starts, +x the way it faces, +z up.
"""
import math

import numpy as np

RES = 0.1


class World:
    def __init__(self, walls, x0, y0, floor_z, ceiling_z):
        self.walls = np.asarray(walls, dtype=bool)     # [i, j], i along x
        self.x0, self.y0 = x0, y0                      # world coords of cell (0, 0) corner
        self.floor_z, self.ceiling_z = floor_z, ceiling_z

    def occupied(self, x, y):
        i = np.floor((x - self.x0) / RES).astype(int)
        j = np.floor((y - self.y0) / RES).astype(int)
        inside = (i >= 0) & (i < self.walls.shape[0]) & (j >= 0) & (j < self.walls.shape[1])
        out = np.ones(i.shape, dtype=bool)             # outside the grid is solid
        out[inside] = self.walls[i[inside], j[inside]]
        return out

    def hits_wall(self, x, y):
        return bool(self.occupied(np.array([x]), np.array([y]))[0])


def two_rooms(door_m=1.0):
    """Room A 5 x 4 m around the start, a wall with a door, room B 4 x 4 m.
    x from -2 to 7.4, y from -2 to 2. Start at (0, 0)."""
    nx, ny = 98, 44
    w = np.zeros((nx, ny), dtype=bool)
    w[:2, :] = w[-2:, :] = True
    w[:, :2] = w[:, -2:] = True
    w[50:54, :] = True                                  # dividing wall, x 3.0 .. 3.4
    half = int(round(door_m / RES / 2))
    w[50:54, ny // 2 - half:ny // 2 + half] = False     # door centred on y = 0
    return World(w, -2.0, -2.2, floor_z=-0.25, ceiling_z=2.75)


def one_room_pillar():
    """Single 7 x 5 m room with a 0.6 m pillar ahead of the start."""
    nx, ny = 74, 54
    w = np.zeros((nx, ny), dtype=bool)
    w[:2, :] = w[-2:, :] = True
    w[:, :2] = w[:, -2:] = True
    w[40:46, 24:30] = True                              # pillar near (2.3, 0)
    return World(w, -2.0, -2.7, floor_z=-0.25, ceiling_z=2.75)


WORLDS = {'two_rooms': two_rooms, 'one_room_pillar': one_room_pillar}


def ray_directions(az_step_deg=2.0, el_min_deg=-7.0, el_max_deg=52.0, el_count=14):
    """Unit vectors covering the Mid-360 field of view (360 x -7..+52 deg)."""
    az = np.deg2rad(np.arange(0.0, 360.0, az_step_deg))
    el = np.deg2rad(np.linspace(el_min_deg, el_max_deg, el_count))
    a, e = np.meshgrid(az, el, indexing='ij')
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=-1).reshape(-1, 3)


def scan(world, origin, yaw, dirs, max_range=12.0, blind=0.5, step=0.05):
    """World-frame hit points (n, 3) of one scan taken at origin with heading yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.stack([c * dirs[:, 0] - s * dirs[:, 1], s * dirs[:, 0] + c * dirs[:, 1], dirs[:, 2]], axis=1)
    o = np.asarray(origin, dtype=float)
    alive = np.ones(len(d), dtype=bool)
    hit = np.full(len(d), np.nan)
    for r in np.arange(blind, max_range, step):
        idx = np.nonzero(alive)[0]
        if not len(idx):
            break
        p = o + d[idx] * r
        h = (p[:, 2] <= world.floor_z) | (p[:, 2] >= world.ceiling_z) | world.occupied(p[:, 0], p[:, 1])
        hit[idx[h]] = r
        alive[idx[h]] = False
    ok = ~np.isnan(hit)
    return (o + d[ok] * hit[ok, None]).astype(np.float32)
