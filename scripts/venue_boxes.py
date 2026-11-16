#!/usr/bin/env python3
# filepath: scripts/venue_boxes.py
"""Derives every box in the stack from config/venue.yaml (operator decision
D5, 2026-11-16) and writes them as one ROS parameter file that the launch
files load last.

  room (measured)  >  geofence hard box  >  soft band  >  planning box
  map bounds = room + pad;  EV guard radius covers the whole room with margin

Usage:
  scripts/venue_boxes.py                      # print the derived boxes, check them
  scripts/venue_boxes.py --write              # also write config/venue_params.yaml
  scripts/venue_boxes.py --venue F --out F
Exit code 1 if the venue cannot hold a consistent set of boxes.
"""
import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'src', 'drone_bringup', 'config')
INFLATION = 0.4      # planner/explorer obstacle_inflation
RESOLUTION = 0.2


def derive(doc):
    v, m = doc['venue'], doc['margins']
    z_fly = float(doc.get('z_fly', 1.3))
    soft = float(m['soft_m'])
    fence = dict(                                    # ENU
        x_min=v['x_min'] + m['wall_m'], x_max=v['x_max'] - m['wall_m'],
        y_min=v['y_min'] + m['wall_m'], y_max=v['y_max'] - m['wall_m'],
        z_min=v['floor_z'] - 0.3,                    # tolerance for sitting on the ground
        z_max=v['ceiling_z'] - m['ceiling_m'])
    eps = 0.05
    plan = dict(
        x_min=fence['x_min'] + soft + eps, x_max=fence['x_max'] - soft - eps,
        y_min=fence['y_min'] + soft + eps, y_max=fence['y_max'] - soft - eps,
        z_min=max(fence['z_min'] + soft, v['floor_z'] + INFLATION + 0.05),
        z_max=fence['z_max'] - soft)
    pad = float(m['map_pad_m'])
    mp = dict(
        x_min=v['x_min'] - pad, x_max=v['x_max'] + pad,
        y_min=v['y_min'] - pad, y_max=v['y_max'] + pad,
        z_min=v['floor_z'] - 0.3, z_max=v['ceiling_z'] + 0.3)
    far = max(math.sqrt(x * x + y * y + z * z)
              for x in (v['x_min'], v['x_max']) for y in (v['y_min'], v['y_max'])
              for z in (v['floor_z'], v['ceiling_z']))
    radius = max(20.0, round(1.5 * far + 5.0, 1))
    r = {k: {a: round(float(b), 3) for a, b in d.items()}
         for k, d in dict(fence_enu=fence, plan=plan, map=mp).items()}
    r.update(soft=soft, z_fly=z_fly, ev_radius=radius)
    # NED for the watchdog: N = ENU y (startup left), E = ENU x (startup forward), D = -z
    f = r['fence_enu']
    r['fence_ned'] = dict(x_min=f['y_min'], x_max=f['y_max'], y_min=f['x_min'],
                          y_max=f['x_max'], z_min=-f['z_max'], z_max=-f['z_min'])
    return r


def problems(r):
    """Why this venue cannot be flown as configured; [] if consistent."""
    e = []
    p, f = r['plan'], r['fence_enu']
    for a in 'xyz':
        if not p[f'{a}_min'] < p[f'{a}_max']:
            e.append(f'planning box is empty along {a}: the room is too small for the margins')
        if not f[f'{a}_min'] < f[f'{a}_max']:
            e.append(f'geofence is empty along {a}')
    if e:
        return e
    for a in 'xy':
        if p[f'{a}_max'] - p[f'{a}_min'] < 2 * INFLATION + 2 * RESOLUTION:
            e.append(f'planning box is narrower than {2 * INFLATION + 2 * RESOLUTION:.1f} m '
                     f'along {a}: no goal fits')
    for a in 'xy':
        if not p[f'{a}_min'] + INFLATION <= 0.0 <= p[f'{a}_max'] - INFLATION:
            e.append(f'the start mark (0, 0) is not inside the planning box along {a} with '
                     f'{INFLATION} m to spare: start further from the walls '
                     f'(planning box {a}[{p[f"{a}_min"]}, {p[f"{a}_max"]}])')
    z_l = (math.floor(r['z_fly'] / RESOLUTION + 1e-6) + 0.5) * RESOLUTION
    if not p['z_min'] + INFLATION - 1e-6 <= z_l <= p['z_max'] - INFLATION + 1e-6:
        e.append(f'z_fly {r["z_fly"]} (layer {z_l:.1f}) is outside '
                 f'[{p["z_min"] + INFLATION:.2f}, {p["z_max"] - INFLATION:.2f}]: '
                 'ceiling too low for the margins, or change z_fly')
    return e


def params_file(r):
    p, mp, n = r['plan'], r['map'], r['fence_ned']
    box = dict(p)
    return {
        'map_interface': {'ros__parameters': dict(mp)},
        'go_to_with_avoidance': {'ros__parameters': dict(
            box, obstacle_inflation=INFLATION, resolution=RESOLUTION)},
        'frontier_explorer': {'ros__parameters': dict(
            box, obstacle_inflation=INFLATION, resolution=RESOLUTION, z_fly=r['z_fly'])},
        'geofence_watchdog': {'ros__parameters': dict(n, soft_margin=r['soft'])},
        'px4_odom_bridge': {'ros__parameters': {'ev_max_radius_m': r['ev_radius']}},
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--venue', default=os.path.join(CFG, 'venue.yaml'))
    ap.add_argument('--out', default=os.path.join(CFG, 'venue_params.yaml'))
    ap.add_argument('--write', action='store_true')
    args = ap.parse_args(argv)
    with open(args.venue) as fh:
        r = derive(yaml.safe_load(fh))
    for name in ('map', 'fence_enu', 'plan'):
        b = r[name]
        print(f'{name:10s} ENU x[{b["x_min"]:6.2f},{b["x_max"]:6.2f}] '
              f'y[{b["y_min"]:6.2f},{b["y_max"]:6.2f}] z[{b["z_min"]:6.2f},{b["z_max"]:6.2f}]')
    n = r['fence_ned']
    print(f'{"fence":10s} NED x[{n["x_min"]:6.2f},{n["x_max"]:6.2f}] '
          f'y[{n["y_min"]:6.2f},{n["y_max"]:6.2f}] z[{n["z_min"]:6.2f},{n["z_max"]:6.2f}] '
          f'soft {r["soft"]}')
    print(f'EV guard radius {r["ev_radius"]} m, z_fly {r["z_fly"]} m')
    errs = problems(r)
    for e in errs:
        print(f'PROBLEM: {e}', file=sys.stderr)
    if errs:
        return 1
    if args.write:
        with open(args.out, 'w') as fh:
            fh.write('# filepath: src/drone_bringup/config/venue_params.yaml\n'
                     '# GENERATED by scripts/venue_boxes.py from config/venue.yaml. '
                     'Do not edit.\n')
            yaml.safe_dump(params_file(r), fh, sort_keys=False)
        print(f'wrote {os.path.relpath(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
