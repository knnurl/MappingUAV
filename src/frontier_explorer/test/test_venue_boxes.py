# filepath: src/frontier_explorer/test/test_venue_boxes.py
"""scripts/venue_boxes.py must produce boxes that pass the explorer's own
consistency checks C2-C4 and its range validation, for any venue it accepts."""
import dataclasses
import importlib.util
import pathlib

import pytest
import yaml

from frontier_explorer.config import Params, check_fence, check_map, check_shim, validate

ROOT = pathlib.Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('venue_boxes', ROOT / 'scripts' / 'venue_boxes.py')
vb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vb)

MARGINS = dict(wall_m=0.5, ceiling_m=0.5, soft_m=0.5, map_pad_m=1.0)


def venue(x0=-2.0, x1=5.0, y0=-2.5, y1=2.5, floor=-0.25, ceil=2.75, z_fly=1.3):
    return dict(venue=dict(x_min=x0, x_max=x1, y_min=y0, y_max=y1,
                           floor_z=floor, ceiling_z=ceil), margins=MARGINS, z_fly=z_fly)


@pytest.mark.parametrize('doc', [
    venue(), venue(-3, 12, -4, 6, -0.3, 3.5), venue(-2, 2.5, -2, 2, -0.25, 2.6, z_fly=1.1),
    venue(-10, 25, -8, 8, -0.25, 6.0, z_fly=1.5),
])
def test_derived_boxes_pass_every_explorer_check(doc):
    r = vb.derive(doc)
    assert vb.problems(r) == []
    f = vb.params_file(r)
    p = dataclasses.replace(Params(), **f['frontier_explorer']['ros__parameters'],
                            max_grid_cells=10 ** 6)
    assert validate(p) == []
    assert check_shim(p, f['go_to_with_avoidance']['ros__parameters']) == []
    m = dict(f['map_interface']['ros__parameters'], map_backend='cpu_grid',
             resolution=0.2, max_insert_range=8.0)
    assert check_map(p, m) == []
    assert check_fence(p, f['geofence_watchdog']['ros__parameters']) == []
    assert f['px4_odom_bridge']['ros__parameters']['ev_max_radius_m'] >= 20.0


def test_room_too_small_or_ceiling_too_low_is_reported():
    assert vb.problems(vb.derive(venue(-0.5, 0.5, -0.5, 0.5)))
    assert vb.problems(vb.derive(venue(ceil=1.6)))


def test_start_mark_too_close_to_a_wall_is_reported():
    errs = vb.problems(vb.derive(venue(x0=-1.0)))
    assert any('start mark' in e for e in errs)


def test_large_venue_raises_the_ev_radius_beyond_20_m():
    r = vb.derive(venue(-10, 40, -8, 8, -0.25, 6.0))
    assert r['ev_radius'] > 40.0


def test_committed_venue_params_match_the_committed_venue():
    cfg = ROOT / 'src' / 'drone_bringup' / 'config'
    r = vb.derive(yaml.safe_load((cfg / 'venue.yaml').read_text()))
    assert yaml.safe_load((cfg / 'venue_params.yaml').read_text()) == vb.params_file(r)
