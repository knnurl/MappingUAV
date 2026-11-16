# filepath: src/frontier_explorer/frontier_explorer/config.py
"""Parameters, range validation (C1) and cross-node consistency checks C2-C6
(spec §5, §6). No ROS imports.

Additions to spec v3.1 (operator decisions 2026-11-16): the battery interlock
(I10), the px4_odom_bridge state interlock (I11) and the planner status topic.
"""
import math
from dataclasses import dataclass, fields

from planner_shim.shim_core import EnuBox

from .grid import EPS, Grid


@dataclass
class Params:
    # 5.1 box, grid, band
    x_min: float = -4.5
    x_max: float = 4.5
    y_min: float = -4.5
    y_max: float = 4.5
    z_min: float = 0.2
    z_max: float = 2.3
    obstacle_inflation: float = 0.4
    resolution: float = 0.2
    z_fly: float = 1.3
    band_half_layers: int = 0
    max_grid_cells: int = 40000
    # 5.2 map sampling
    cycle_period_s: float = 2.0
    max_points_per_request: int = 4096
    query_timeout_s: float = 1.0
    map_fail_max: int = 3
    map_ready_min_free_cells: int = 50
    wait_map_timeout_s: float = 30.0
    # 5.3 frontier extraction
    min_unknown_region_cells: int = 9
    min_cluster_cells: int = 3
    max_clusters: int = 32
    # 5.4 traversability and viewpoints
    traverse_clearance_m: float = 0.4
    goal_clearance_m: float = 0.6
    unknown_standoff_m: float = 0.6
    allow_tight_goals: bool = True
    view_range_m: float = 3.0
    candidate_stride_cells: int = 2
    max_candidates_per_cluster: int = 256
    max_gain_samples: int = 64
    start_snap_radius_m: float = 0.6
    # 5.5 selection
    distance_weight: float = 1.0
    switch_margin: float = 2.0
    # 5.6 goal lifecycle
    reach_tolerance_xy_m: float = 0.3
    reach_tolerance_z_m: float = 0.3
    nominal_speed_m_s: float = 0.5
    goal_timeout_factor: float = 3.0
    goal_timeout_min_s: float = 20.0
    progress_window_s: float = 15.0
    progress_min_m: float = 0.3
    start_motion_timeout_s: float = 5.0
    reject_window_s: float = 1.0
    blacklist_radius_m: float = 1.0
    visited_radius_m: float = 0.6
    blacklist_max_entries: int = 64
    blacklist_ttl_s: float = 0.0
    # 5.7 termination
    done_confirm_cycles: int = 3
    max_mission_s: float = 900.0
    on_done: str = 'hold'
    # 5.8 interlocks
    pose_timeout_s: float = 0.5
    platform_info_timeout_s: float = 2.0
    lio_heartbeat_timeout_s: float = 3.0
    fence_heartbeat_timeout_s: float = 1.5
    frame_divergence_max_m: float = 0.5
    frame_divergence_persist_s: float = 1.0
    frame_check_max_dt_s: float = 0.2
    remote_param_timeout_s: float = 2.0
    # additions 2026-11-16
    battery_min_fraction: float = 0.25      # I10: below this the mission ends
    battery_timeout_s: float = 10.0
    battery_required: bool = True           # False: fly without a battery topic
    bridge_heartbeat_timeout_s: float = 2.0  # I11 (publisher 2 Hz)
    # 5.9 interfaces and diagnostics
    as2_namespace: str = 'drone0'
    goal_frame_id: str = 'odom'
    shim_node: str = '/go_to_with_avoidance'
    map_node: str = '/map_interface'
    fence_node: str = '/geofence_watchdog'
    publish_markers: bool = False
    max_plan_time_s: float = 1.0

    @property
    def box(self):
        return EnuBox(self.x_min, self.x_max, self.y_min, self.y_max,
                      self.z_min, self.z_max)

    def grid(self):
        return Grid.from_box(self.box, self.resolution, self.z_fly)

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


def validate(p: Params):
    """C1: every range of spec §5, plus C5 and C6. Returns a list of error
    strings, each starting with the parameter name; empty when valid."""
    e = []

    def need(ok, name, rule):
        if not ok:
            e.append(f'{name}: {rule} (got {getattr(p, name)!r})')

    for name in Params.names():
        v = getattr(p, name)
        if isinstance(v, float) and not math.isfinite(v):
            e.append(f'{name}: must be finite (got {v!r})')
    if e:
        return e
    need(p.x_min < p.x_max, 'x_min', 'must be < x_max')
    need(p.y_min < p.y_max, 'y_min', 'must be < y_max')
    need(p.z_min < p.z_max, 'z_min', 'must be < z_max')
    need(p.obstacle_inflation >= 0, 'obstacle_inflation', 'must be >= 0')
    need(p.resolution > 0, 'resolution', 'must be > 0')
    need(p.band_half_layers == 0, 'band_half_layers', 'must be 0 in Phase 1')
    need(p.max_grid_cells >= 1, 'max_grid_cells', 'must be >= 1')
    need(0.5 <= p.cycle_period_s <= 10, 'cycle_period_s', 'must be in [0.5, 10]')
    need(256 <= p.max_points_per_request <= 65536, 'max_points_per_request',
         'must be in [256, 65536]')
    need(0 < p.query_timeout_s < p.cycle_period_s, 'query_timeout_s',
         'must be in (0, cycle_period_s)')
    need(p.map_fail_max >= 1, 'map_fail_max', 'must be >= 1')
    need(p.map_ready_min_free_cells >= 1, 'map_ready_min_free_cells', 'must be >= 1')
    need(p.wait_map_timeout_s > 0, 'wait_map_timeout_s', 'must be > 0')
    need(p.min_unknown_region_cells >= 1, 'min_unknown_region_cells', 'must be >= 1')
    need(p.min_cluster_cells >= 1, 'min_cluster_cells', 'must be >= 1')
    need(p.max_clusters >= 1, 'max_clusters', 'must be >= 1')
    need(p.traverse_clearance_m >= 0, 'traverse_clearance_m', 'must be >= 0')
    need(p.goal_clearance_m >= p.traverse_clearance_m, 'goal_clearance_m',
         'must be >= traverse_clearance_m')
    need(p.unknown_standoff_m >= p.traverse_clearance_m, 'unknown_standoff_m',
         'must be >= traverse_clearance_m')
    need(p.view_range_m > 0, 'view_range_m', 'must be > 0')
    need(p.candidate_stride_cells >= 1, 'candidate_stride_cells', 'must be >= 1')
    need(p.max_candidates_per_cluster >= 1, 'max_candidates_per_cluster', 'must be >= 1')
    need(p.max_gain_samples >= 1, 'max_gain_samples', 'must be >= 1')
    need(p.start_snap_radius_m >= 0, 'start_snap_radius_m', 'must be >= 0')
    need(p.distance_weight >= 0, 'distance_weight', 'must be >= 0')
    need(p.switch_margin >= 0, 'switch_margin', 'must be >= 0')
    need(p.visited_radius_m > 0, 'visited_radius_m', 'must be > 0')
    need(0 < p.reach_tolerance_xy_m < p.visited_radius_m, 'reach_tolerance_xy_m',
         'must be in (0, visited_radius_m)')
    need(p.reach_tolerance_z_m > 0, 'reach_tolerance_z_m', 'must be > 0')
    need(0 < p.nominal_speed_m_s <= 1.0, 'nominal_speed_m_s', 'must be in (0, 1.0]')
    need(p.goal_timeout_factor >= 1, 'goal_timeout_factor', 'must be >= 1')
    need(p.goal_timeout_min_s > 0, 'goal_timeout_min_s', 'must be > 0')
    need(p.progress_window_s > p.cycle_period_s, 'progress_window_s',
         'must be > cycle_period_s')
    need(p.progress_min_m > 0, 'progress_min_m', 'must be > 0')
    need(p.start_motion_timeout_s > 0, 'start_motion_timeout_s', 'must be > 0')
    need(p.reject_window_s > 0, 'reject_window_s', 'must be > 0')
    need(p.blacklist_radius_m > 0, 'blacklist_radius_m', 'must be > 0')
    need(p.blacklist_max_entries >= 1, 'blacklist_max_entries', 'must be >= 1')
    need(p.blacklist_ttl_s >= 0, 'blacklist_ttl_s', 'must be >= 0')
    need(p.done_confirm_cycles >= 1, 'done_confirm_cycles', 'must be >= 1')
    need(p.max_mission_s > 0, 'max_mission_s', 'must be > 0')
    need(p.on_done in ('hold', 'return_start'), 'on_done', "must be 'hold' or 'return_start'")
    need(p.pose_timeout_s > 0, 'pose_timeout_s', 'must be > 0')
    need(p.platform_info_timeout_s > 0, 'platform_info_timeout_s', 'must be > 0')
    need(p.lio_heartbeat_timeout_s > 1.0, 'lio_heartbeat_timeout_s', 'must be > 1.0')
    need(p.fence_heartbeat_timeout_s > 0.5, 'fence_heartbeat_timeout_s', 'must be > 0.5')
    need(p.frame_divergence_max_m > 0, 'frame_divergence_max_m', 'must be > 0')
    need(p.frame_divergence_persist_s >= 0, 'frame_divergence_persist_s', 'must be >= 0')
    need(p.frame_check_max_dt_s > 0, 'frame_check_max_dt_s', 'must be > 0')
    need(p.remote_param_timeout_s > 0, 'remote_param_timeout_s', 'must be > 0')
    need(0 < p.battery_min_fraction < 1, 'battery_min_fraction', 'must be in (0, 1)')
    need(p.battery_timeout_s > 0, 'battery_timeout_s', 'must be > 0')
    need(p.bridge_heartbeat_timeout_s > 0.5, 'bridge_heartbeat_timeout_s', 'must be > 0.5')
    need(p.max_plan_time_s > 0, 'max_plan_time_s', 'must be > 0')
    if e:
        return e
    g = p.grid()
    if not (p.z_min + p.obstacle_inflation - EPS <= g.z_L
            <= p.z_max - p.obstacle_inflation + EPS):
        e.append(f'z_fly: layer centre {g.z_L:.3f} outside '
                 f'[z_min + inflation, z_max - inflation] (C5)')
    if g.nx < 1 or g.ny < 1:
        e.append('resolution: planning box holds no cell')
    elif g.nx * g.ny > p.max_grid_cells:
        e.append(f'max_grid_cells: grid {g.nx} x {g.ny} exceeds it (C6)')
    return e


def ned_box_to_enu(x_min, x_max, y_min, y_max, z_min, z_max):
    """Geofence NED box -> ENU (x_min, x_max, y_min, y_max, z_min, z_max)."""
    return (y_min, y_max, x_min, x_max, -z_max, -z_min)


def box_inside(inner, outer, shrink):
    ix0, ix1, iy0, iy1, iz0, iz1 = inner
    ox0, ox1, oy0, oy1, oz0, oz1 = outer
    t = 1e-6
    return (ix0 >= ox0 + shrink - t and ix1 <= ox1 - shrink + t
            and iy0 >= oy0 + shrink - t and iy1 <= oy1 - shrink + t
            and iz0 >= oz0 + shrink - t and iz1 <= oz1 - shrink + t)


def _box6(d):
    return tuple(float(d[k]) for k in ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'))


def check_shim(p: Params, shim: dict):
    """C2: the planner's box and inflation equal ours within 1e-6."""
    e = []
    for k in ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'obstacle_inflation'):
        if k not in shim or shim[k] is None:
            e.append(f'C2: planner parameter {k} missing')
        elif abs(float(shim[k]) - getattr(p, k)) > 1e-6:
            e.append(f'C2: planner {k}={shim[k]} differs from explorer {getattr(p, k)}')
    return e


def check_map(p: Params, m: dict):
    """C3: backend, resolution, box inside the map bounds, view range."""
    e = []
    need = ('map_backend', 'resolution', 'max_insert_range',
            'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')
    missing = [k for k in need if m.get(k) is None]
    if missing:
        return [f'C3: map parameter {k} missing' for k in missing]
    if m['map_backend'] != 'cpu_grid':
        e.append(f"C3: map_backend is {m['map_backend']!r}, need 'cpu_grid'")
    if abs(float(m['resolution']) - p.resolution) > 1e-6:
        e.append(f"C3: map resolution {m['resolution']} differs from {p.resolution}")
    if not box_inside(_box6(p.__dict__), _box6(m), p.resolution):
        e.append('C3: planning box is not inside the map bounds shrunk by one cell')
    if p.view_range_m > float(m['max_insert_range']) + 1e-6:
        e.append(f"C3: view_range_m {p.view_range_m} exceeds map max_insert_range "
                 f"{m['max_insert_range']}")
    return e


def check_fence(p: Params, f: dict):
    """C4: planning box inside the ENU geofence shrunk by its soft margin."""
    need = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'soft_margin')
    missing = [k for k in need if f.get(k) is None]
    if missing:
        return [f'C4: geofence parameter {k} missing' for k in missing]
    enu = ned_box_to_enu(*_box6(f))
    if not box_inside(_box6(p.__dict__), enu, float(f['soft_margin'])):
        return [f'C4: planning box {_box6(p.__dict__)} is not inside the geofence '
                f'(ENU {enu}) shrunk by soft_margin {f["soft_margin"]}']
    return []
