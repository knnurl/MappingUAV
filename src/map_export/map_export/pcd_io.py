# filepath: src/map_export/map_export/pcd_io.py
"""Minimal binary PCD writer/reader + voxel downsample (WP-C map_export).
Pure Python + struct: unit-testable, no PCL dependency on the vehicle."""
import struct

_HEADER = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA binary
"""


def write_pcd(path, points):
    """points: iterable of (x, y, z) floats."""
    pts = list(points)
    with open(path, 'wb') as f:
        f.write(_HEADER.format(n=len(pts)).encode('ascii'))
        for p in pts:
            f.write(struct.pack('<fff', *p))
    return len(pts)


def read_pcd(path):
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode('ascii', errors='strict').strip()
            if line.startswith('POINTS'):
                n = int(line.split()[1])
            if line.startswith('DATA'):
                if line != 'DATA binary':
                    raise ValueError(f'unsupported PCD: {line}')
                break
        raw = f.read(12 * n)
    return [struct.unpack_from('<fff', raw, 12 * i) for i in range(n)]


def voxel_downsample(points, voxel=0.05):
    """Keep the centroid of each voxel cell. Deterministic, order-independent."""
    cells = {}
    for (x, y, z) in points:
        key = (int(x // voxel), int(y // voxel), int(z // voxel))
        acc = cells.get(key)
        if acc is None:
            cells[key] = [x, y, z, 1]
        else:
            acc[0] += x
            acc[1] += y
            acc[2] += z
            acc[3] += 1
    return [(sx / c, sy / c, sz / c) for (sx, sy, sz, c) in cells.values()]
