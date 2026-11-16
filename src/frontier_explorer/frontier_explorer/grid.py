# filepath: src/frontier_explorer/frontier_explorer/grid.py
"""Band grid geometry and QueryMap sampling (spec §7.2 - §7.4). No ROS imports.

Ported from tools/frontier_prototype/proto.py, the verified reference for
spec §7. Cells are map voxels sampled at their centres.
"""
import math
from dataclasses import dataclass

import numpy as np

from planner_shim.shim_core import EnuBox

FREE, OCC, UNK = 0, 1, 2
EPS = 1e-6

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
