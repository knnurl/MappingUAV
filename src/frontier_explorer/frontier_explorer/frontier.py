# filepath: src/frontier_explorer/frontier_explorer/frontier.py
"""Unknown-region filter, frontier mask, clusters, distance to unknown
(spec §7.5 - §7.7, §7.9). No ROS imports. Ported from the verified prototype.
"""
import numpy as np
from scipy import ndimage

from .grid import FREE, UNK

S4 = ndimage.generate_binary_structure(2, 1)
S8 = ndimage.generate_binary_structure(2, 2)

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


def dist_to_unknown(unk_eff, res):
    if not unk_eff.any():
        return np.full(unk_eff.shape, np.inf)
    return ndimage.distance_transform_edt(~unk_eff) * res
