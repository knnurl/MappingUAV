# filepath: src/frontier_explorer/test/test_config.py
import dataclasses

import pytest

from frontier_explorer.config import (
    Params, box_inside, check_fence, check_map, check_shim, ned_box_to_enu,
    validate,
)


def bad(**kw):
    return validate(dataclasses.replace(Params(), **kw))


def test_defaults_are_valid():
    assert validate(Params()) == []


@pytest.mark.parametrize('name,value', [                       # T-C01
    ('x_min', 5.0), ('z_min', 3.0), ('obstacle_inflation', -0.1),
    ('resolution', 0.0), ('max_grid_cells', 0), ('cycle_period_s', 0.2),
    ('cycle_period_s', 11.0), ('max_points_per_request', 100),
    ('query_timeout_s', 2.0), ('map_fail_max', 0), ('wait_map_timeout_s', 0.0),
    ('min_unknown_region_cells', 0), ('min_cluster_cells', 0), ('max_clusters', 0),
    ('traverse_clearance_m', -1.0), ('goal_clearance_m', 0.3),
    ('unknown_standoff_m', 0.3), ('view_range_m', 0.0),
    ('candidate_stride_cells', 0), ('max_gain_samples', 0),
    ('distance_weight', -1.0), ('switch_margin', -1.0),
    ('reach_tolerance_xy_m', 0.6), ('reach_tolerance_z_m', 0.0),
    ('nominal_speed_m_s', 1.5), ('goal_timeout_factor', 0.5),
    ('progress_window_s', 2.0), ('progress_min_m', 0.0),
    ('blacklist_radius_m', 0.0), ('blacklist_ttl_s', -1.0),
    ('done_confirm_cycles', 0), ('max_mission_s', 0.0), ('on_done', 'land'),
    ('pose_timeout_s', 0.0), ('lio_heartbeat_timeout_s', 1.0),
    ('fence_heartbeat_timeout_s', 0.5), ('frame_divergence_max_m', 0.0),
    ('battery_min_fraction', 1.0), ('battery_min_fraction', 0.0),
    ('bridge_heartbeat_timeout_s', 0.5), ('max_plan_time_s', 0.0),
    ('x_max', float('nan')),
])
def test_range_violation_is_rejected_with_its_name(name, value):
    errors = bad(**{name: value})
    assert errors and any(e.startswith(name) for e in errors), errors


def test_band_half_layers_must_be_zero():                      # T-C06
    assert any('band_half_layers' in e for e in bad(band_half_layers=1))


def test_layer_centre_must_respect_inflation():                # T-C05
    assert any('C5' in e for e in bad(z_fly=0.3))
    assert any('C5' in e for e in bad(z_fly=2.2))
    assert bad(z_fly=1.2) == []       # snaps to the 1.3 m layer


def test_grid_cell_limit():                                    # C6
    assert any('C6' in e for e in bad(max_grid_cells=2000))


SHIM = dict(x_min=-4.5, x_max=4.5, y_min=-4.5, y_max=4.5, z_min=0.2, z_max=2.3,
            obstacle_inflation=0.4)


@pytest.mark.parametrize('key', list(SHIM))                    # T-C02
def test_each_planner_mismatch_fails_c2(key):
    assert check_shim(Params(), SHIM) == []
    assert check_shim(Params(), {**SHIM, key: SHIM[key] + 0.1})
    missing = dict(SHIM)
    del missing[key]
    assert check_shim(Params(), missing)


MAP = dict(map_backend='cpu_grid', resolution=0.2, max_insert_range=8.0,
           x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0, z_min=-0.5, z_max=3.0)


def test_c3():                                                 # T-C03
    p = Params()
    assert check_map(p, MAP) == []
    assert check_map(p, {**MAP, 'map_backend': 'nvblox'})
    assert check_map(p, {**MAP, 'resolution': 0.1})
    assert check_map(p, {**MAP, 'x_max': 4.6})      # box not inside by one cell
    assert check_map(p, {**MAP, 'max_insert_range': 2.0})
    assert check_map(p, {k: v for k, v in MAP.items() if k != 'resolution'})


def test_ned_to_enu_asymmetric_box():                          # T-C04
    assert ned_box_to_enu(-1, 2, -3, 4, -2.5, 0.3) == (-3, 4, -1, 2, -0.3, 2.5)


def test_c4_fails_on_the_branch_defaults_and_passes_when_sized():
    p = Params()
    gate4_fence = dict(x_min=-1.5, x_max=1.5, y_min=-1.5, y_max=1.5,
                       z_min=-2.5, z_max=0.3, soft_margin=0.5)
    assert check_fence(p, gate4_fence)             # OI-2 reproduced
    sized = dict(x_min=-5.0, x_max=5.0, y_min=-5.0, y_max=5.0,
                 z_min=-2.8, z_max=0.3, soft_margin=0.5)
    assert check_fence(p, sized) == []
    assert check_fence(p, {**sized, 'z_min': -2.7})   # ceiling margin too thin
    assert check_fence(p, {k: v for k, v in sized.items() if k != 'soft_margin'})


def test_box_inside():
    assert box_inside((0, 1, 0, 1, 0, 1), (-1, 2, -1, 2, -1, 2), 1.0)
    assert not box_inside((0, 1, 0, 1, 0, 1), (-1, 2, -1, 2, -1, 2), 1.1)
