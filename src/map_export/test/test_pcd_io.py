# filepath: src/map_export/test/test_pcd_io.py
"""map_export unit tests: PCD roundtrip and voxel downsample."""
import os

import pytest

from map_export.pcd_io import write_pcd, read_pcd, voxel_downsample


def test_pcd_roundtrip(tmp_path):
    pts = [(0.0, 0.0, 0.0), (1.5, -2.25, 3.125), (-0.1, 0.2, -0.3)]
    p = str(tmp_path / 'out.pcd')
    assert write_pcd(p, pts) == 3
    back = read_pcd(p)
    assert len(back) == 3
    for a, b in zip(pts, back):
        assert a == pytest.approx(b, abs=1e-6)


def test_pcd_empty(tmp_path):
    p = str(tmp_path / 'empty.pcd')
    write_pcd(p, [])
    assert read_pcd(p) == []


def test_voxel_downsample_merges_and_centroids():
    pts = [(0.01, 0.01, 0.01), (0.03, 0.03, 0.03),   # same 5 cm voxel
           (1.0, 1.0, 1.0)]                           # far away
    out = voxel_downsample(pts, voxel=0.05)
    assert len(out) == 2
    merged = min(out)                                 # the near-origin one
    assert merged == pytest.approx((0.02, 0.02, 0.02), abs=1e-6)


def test_voxel_downsample_preserves_isolated_points():
    pts = [(x * 1.0, 0.0, 0.0) for x in range(10)]
    assert len(voxel_downsample(pts, voxel=0.05)) == 10


def test_postprocess_cli(tmp_path):
    from map_export.postprocess import main
    write_pcd(str(tmp_path / 'chunk_0000.pcd'),
              [(0.01, 0.0, 0.0), (0.02, 0.0, 0.0)])
    write_pcd(str(tmp_path / 'chunk_0001.pcd'), [(2.0, 2.0, 2.0)])
    assert main([str(tmp_path), '--voxel', '0.05']) == 0
    merged = read_pcd(str(tmp_path / 'merged_0.05.pcd'))
    assert len(merged) == 2


def test_postprocess_cli_no_chunks(tmp_path):
    from map_export.postprocess import main
    assert main([str(tmp_path)]) == 1
