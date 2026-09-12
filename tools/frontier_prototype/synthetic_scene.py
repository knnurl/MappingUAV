# filepath: tools/frontier_prototype/synthetic_scene.py
# VERBATIM COPY of wp-c-mapping@a373313:tools/replay_tests/synthetic_scene.py (used by bench_query.py).
# filepath: tools/replay_tests/synthetic_scene.py
"""Procedural room+pillar point-cloud generator (WP-C).

This is the interim test asset while no Gate 2/3 bags exist (masterplan §5:
"synthetic clouds until then, logged as debt"). Deliberately NOT the deferred
SITL work: no physics, no sensor model beyond range sampling — just
world-frame surface points as FAST-LIO would output on /cloud_registered.

Scene: rectangular room with four walls, floor, and vertical square pillars.
All coordinates odom/ENU, metres.
"""
import math
from dataclasses import dataclass, field


@dataclass
class Scene:
    # Wall planes sit MID-VOXEL at 0.2 m resolution (3.9 -> voxel [3.8, 4.0)):
    # planes exactly on voxel boundaries make endpoint rounding ambiguous.
    x_min: float = -3.9
    x_max: float = 3.9
    y_min: float = -3.9
    y_max: float = 3.9
    wall_height: float = 2.5
    # (cx, cy, half_width) per pillar; faces mid-voxel too
    pillars: list = field(default_factory=lambda: [(2.0, 0.0, 0.25),
                                                   (-1.5, 1.5, 0.25)])


def _frange(a, b, step):
    v = a
    while v <= b + 1e-9:
        yield v
        v += step


def wall_points(scene: Scene, step=0.1):
    pts = []
    for z in _frange(0.0, scene.wall_height, step):
        for x in _frange(scene.x_min, scene.x_max, step):
            pts.append((x, scene.y_min, z))
            pts.append((x, scene.y_max, z))
        for y in _frange(scene.y_min, scene.y_max, step):
            pts.append((scene.x_min, y, z))
            pts.append((scene.x_max, y, z))
    return pts


def floor_points(scene: Scene, step=0.2):
    return [(x, y, 0.0)
            for x in _frange(scene.x_min, scene.x_max, step)
            for y in _frange(scene.y_min, scene.y_max, step)]


def pillar_points(cx, cy, hw, height=2.5, step=0.025):
    # step must stay <= the depth-buffer bin arc length at closest approach,
    # or rays slip between surface samples and carve the interior.
    pts = []
    for z in _frange(0.0, height, step):
        for t in _frange(-hw, hw, step):
            pts.extend([(cx + t, cy - hw, z), (cx + t, cy + hw, z),
                        (cx - hw, cy + t, z), (cx + hw, cy + t, z)])
    return pts


def scene_points(scene: Scene):
    pts = wall_points(scene) + floor_points(scene)
    for (cx, cy, hw) in scene.pillars:
        pts += pillar_points(cx, cy, hw, scene.wall_height)
    return pts


def _segment_hits_aabb(o, p, lo, hi, t_max=0.999):
    """True if segment o->p enters the AABB strictly before reaching p
    (slab test). Endpoints ON the box surface (t ~ 1) do not count."""
    t0, t1 = 0.0, t_max
    for i in range(3):
        d = p[i] - o[i]
        if abs(d) < 1e-12:
            if o[i] < lo[i] or o[i] > hi[i]:
                return False
        else:
            ta = (lo[i] - o[i]) / d
            tb = (hi[i] - o[i]) / d
            if ta > tb:
                ta, tb = tb, ta
            t0 = max(t0, ta)
            t1 = min(t1, tb)
            if t0 > t1:
                return False
    return True


def visible_slice(points, origin, scene: Scene, max_range=8.0):
    """Exact per-scan visibility: a point is visible if within range and its
    line of sight does not pass through any pillar solid. Earlier versions
    used a spherical depth buffer over the sampled points; bin-boundary
    aliasing let background rays slip past obstacle samples and the map's
    raycaster carved obstacle interiors free. Analytic occlusion has no such
    aliasing. Walls/floor sit on the scene boundary and occlude nothing that
    matters at these ranges. Floor points under a pillar footprint are inside
    its AABB and are culled automatically."""
    ox, oy, oz = origin
    boxes = [((cx - hw, cy - hw, 0.0), (cx + hw, cy + hw, scene.wall_height))
             for (cx, cy, hw) in scene.pillars]
    out = []
    for p in points:
        if math.dist(p, origin) > max_range:
            continue
        if any(_segment_hits_aabb(origin, p, lo, hi) for (lo, hi) in boxes):
            continue
        out.append(p)
    return out


def circular_trajectory(radius=1.0, n=20, z=1.0):
    """Poses circling the room centre, as (x, y, z) tuples."""
    return [(radius * math.cos(2 * math.pi * i / n),
             radius * math.sin(2 * math.pi * i / n), z) for i in range(n)]
