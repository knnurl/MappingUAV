# filepath: src/map_export/map_export/postprocess.py
"""Offline map post-processing (WP-C map_export). Run on a WORKSTATION or on
the bench — never during flight autonomy.

  python3 -m map_export.postprocess <session_dir> [--voxel 0.05]

Merges chunk_*.pcd, voxel-downsamples, writes merged_<voxel>.pcd.

Mesh + HBA pose refinement [VERIFY outcome 2026-07-05]: hku-mars/HBA is
ROS1-only upstream — no ROS2/offline CLI port found; meshing needs Open3D or
CloudCompare, neither vehicle-appropriate. Both are therefore WORKSTATION
steps, documented here, not implemented on-vehicle:
  o3d: mesh via pcd -> TSDF/Poisson in a few lines on any x86 machine.
"""
import argparse
import glob
import os
import sys

from .pcd_io import read_pcd, write_pcd, voxel_downsample


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('session_dir')
    ap.add_argument('--voxel', type=float, default=0.05)
    args = ap.parse_args(argv)

    chunks = sorted(glob.glob(os.path.join(args.session_dir, 'chunk_*.pcd')))
    if not chunks:
        print(f'no chunks in {args.session_dir}', file=sys.stderr)
        return 1
    pts = []
    for c in chunks:
        pts.extend(read_pcd(c))
        print(f'read {c} (total {len(pts)})')
    out_pts = voxel_downsample(pts, args.voxel)
    out = os.path.join(args.session_dir, f'merged_{args.voxel:.2f}.pcd')
    write_pcd(out, out_pts)
    print(f'{len(pts)} -> {len(out_pts)} pts @ {args.voxel} m -> {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
