<!-- filepath: docs/wp-f_frontier_explorer_spec.md -->
# WP-F Implementation Spec: `frontier_explorer`

| | |
|---|---|
| Status | **DRAFT v3.1**: verification passes 1 and 2 applied (§18); updated for the recorded DP-1 and DP-2 decisions. For operator review; nothing implemented. |
| Date | 2026-09-12 |
| Work package | WP-F (exploration), DP-2 decision `FRONTIER` |
| Governing docs | `phases3-7_masterplan_consolidated.md` §1 (operating model), §1.2 (frozen contracts), §8 (WP-F); `gates1-4_handover_spec.md` global rules |
| Status cap | **EVIDENCE_ONLY** until `decision_dp2_explore` is set to `FRONTIER`; then **BUILT_UNVERIFIED** (masterplan §8, §1.4). The decision is recorded in `docs/gate_status.yaml` (2026-09-12, operator commit pending), so implementation work is capped at **BUILT_UNVERIFIED**. Closed-loop exploration is untestable without sim or vehicle. |
| Target branch | `wp-f-frontier` (see §1.3) |

## 0. Summary

`frontier_explorer` is a single ROS 2 node that decides **where the drone should go next** to reveal unknown space inside the planning box, at a fixed flight altitude (Phase 1). Each cycle it samples a horizontal slice of the WP-C CPU-grid map through the existing `/map_interface/query` service, extracts **frontier cells** (known-free cells bordering unknown), clusters them, finds a reachable viewpoint per cluster, and publishes the best one as a `PoseStamped` goal to the existing `/go_to_with_avoidance/goal` input of `planner_shim`. EGO-Planner, Aerostack2 and PX4 execute; the geofence watchdog enforces independently.

The node owns only goal selection. It does not map, avoid obstacles, generate trajectories, stream commands, or talk to PX4. It is built from a pure-Python core (no ROS imports, fully unit-testable) and a thin ROS adapter.

## 1. Scope

### 1.1 In scope
- Frontier-based goal selection in a single altitude layer (Phase 1).
- Goal lifecycle: arrival, frontier cleared, no progress, timeout, shim rejection, blacklisting.
- Termination detection and reporting.
- Safety interlocks that stop goal publication (pose, platform, LIO health, geofence, map, frame consistency).
- Operator control (start / pause / resume / stop) and status telemetry.
- Startup configuration-consistency checks across shim, map server and watchdog.
- Phase 2 (varying altitude) extension hooks: parameters reserved and validated, not implemented.

### 1.2 Non-goals [FIXED]
- No mapping. `map_interface` (WP-C) is the only map.
- No obstacle avoidance, trajectory generation, velocity or attitude output. EGO (WP-D) and Aerostack2 (WP-B) own motion.
- No publication to any `/fmu/*` topic; no arming, takeoff, landing or mode changes (masterplan §8: "The exploration planner never talks to PX4 directly").
- No periodic command streaming (SR-6).
- No custom message or service package (IR-2).
- No modification of `planner_shim`, `map_interface`, `geofence_watchdog`, `lio_health_guard` or third-party code inside this WP. Interface gaps are logged as open items (§16).
- No dynamic-obstacle modelling, no multi-drone logic, no sim build (masterplan §10).
- No altitude variation in Phase 1.
- No detection of transparent obstacles (glass). LiDAR cannot see them. This is a venue hazard handled by survey and geofence sizing (§10, H-11).

### 1.3 Preconditions, tokens, status and branch
| Kind | Requirement | State today |
|---|---|---|
| Build precondition (masterplan §8) | WP-C `map_interface` merged | **Not met; waiting.** `map_interface` lives on `wp-c-mapping` and reaches `main` only after the `gate5_as2_behaviors` token (masterplan §1.1 rule 3). Operator decision 2026-09-13 (OI-1): no deviation, so no WP-F implementation code is written until then. |
| Decision precondition | `decision_dp2_explore` set by the human | **Met.** `decision_dp2_explore: FRONTIER` and `decision_dp1_mapping: CPU_GRID` are recorded in `docs/gate_status.yaml` (2026-09-12, operator commit pending). Masterplan amendment A1 (2026-09-13) adds FRONTIER as a DP-2 candidate (OI-7 resolved). |
| Status cap (masterplan §8) | EVIDENCE_ONLY until the decision; BUILT_UNVERIFIED for the explorer afterwards | Implementation commits: BUILT_UNVERIFIED. This spec and `tools/frontier_prototype/` remain EVIDENCE_ONLY. |
| Integration precondition | `gate6_planner_in_loop: CONFIRMED` | PENDING |
| Gate 7 entry | Deliberate watchdog trip test (masterplan §3) | PENDING |
| Commit gate (masterplan §1) | `colcon build` clean, lint clean, all unit tests executed and passing, VD entries opened | Applies to every commit on the branch |

The agent never edits `docs/gate_status.yaml`; only the operator records the decision (masterplan §1.1).

Once `map_interface` and `planner_shim` are on `main` (after their gate tokens), create `wp-f-frontier` from `main`. Fix OI-11 (`wp-d`'s `setup.py` does not install `config/planning/`) on `wp-d-planner` before it merges.

## 2. Conventions and definitions

### 2.1 Frames [FIXED]
- All explorer computation is in the `odom` frame, ENU, metres (masterplan §1.2: planners never consume NED).
- Numeric frame identity assumed by this design:
  `FAST-LIO world (camera_init)` = `map_interface` frame `odom` = Aerostack2 `earth` (identity earth→map→odom statics, `set_map_to_odom` true, no GPS) = PX4 local NED after the `px4_odom_bridge` axis swap.
  The EKF2 external-vision origin offset is **[VERIFY at Gate 3]** (OI-6). A runtime monitor enforces the assumption (SR-9).
- NED→ENU for geofence boxes: `(x_e, y_e, z_e) = (y_n, x_n, −z_n)`. For an NED box:
  `x_e ∈ [y_min_n, y_max_n]`, `y_e ∈ [x_min_n, x_max_n]`, `z_e ∈ [−z_max_n, −z_min_n]`.
- Axis meaning (from `px4_odom_bridge`): ENU +x = vehicle startup forward = PX4 East; ENU +y = startup left = PX4 North.

### 2.2 QueryMap semantics (verified against `wp-c-mapping` source, §18)
- Request `geometry_msgs/Point[] points`. Response `uint8[] occupancy`, `float32[] clearance`, one entry per point, same order.
- `occupancy`: `0` FREE, `1` OCCUPIED, `2` UNKNOWN. **Points outside the map bounds return UNKNOWN.**
- `clearance`: metres to the nearest occupied voxel in **3D**, clamped at `clearance_cap` (default 4.0). `−1.0` = outside bounds or EDT error. Distances are voxel-centre to voxel-centre (§7.8).
- The EDT is built with `unknown_as_occupied: false`, so **unknown space counts as free**. Clearance is optimistic next to unknown space; §7.12 compensates.
- `onQuery` recomputes a dirty distance map **synchronously** before answering. Query latency therefore includes EDT update time after new insertions, and the single-threaded server inserts no clouds while serving.
- Server node name `map_interface`. Declared parameters include `map_backend` (declared unconditionally; the server throws on anything but `cpu_grid`), `x_min … z_max`, `resolution`, `clearance_cap`, `unknown_as_occupied`, `insert_period_s`, `max_insert_range`, `edt_period_s`, `map_publish_period_s`.

### 2.3 Symbols
| Symbol | Meaning |
|---|---|
| `res` | grid resolution = map resolution (0.2 m) |
| `B` | planning box, ENU (`x_min … z_max`), equal to the shim's box |
| `infl` | `obstacle_inflation`, box-face margin used by `check_goal` |
| `G` | the band grid: cells whose centres lie in `B` at layer `z_L` |
| `occ[c]`, `clr[c]` | QueryMap results for cell `c` |
| `N4(c)`, `N8(c)` | 4- and 8-neighbours of `c` within `G` (neighbours outside `G` do not exist) |
| `EPS` | `1e-6`, floating tolerance for index arithmetic |

## 3. Architecture

```
/cloud_registered, /Odometry ──► map_interface ──(/map_interface/query)──► frontier_explorer
                                                                              │
          /<ns>/self_localization/pose, /<ns>/platform/info ─────────────────►│
          /lio_health/state, /geofence/state, /go_to_with_avoidance/alert ───►│
                                                                              │ PoseStamped goal
                                                                              ▼
                                           planner_shim (/go_to_with_avoidance/goal)
                                                  │ check_goal ✓
                                                  ▼
                                               EGO ──► Aerostack2 ──► PX4 offboard
geofence_watchdog ─── independent: hard breach ⇒ NAV_LAND on /fmu/in/vehicle_command ──► PX4
```

### 3.1 Module decomposition
| Module | Responsibility | ROS imports |
|---|---|---|
| `config.py` | Parameter dataclass, range validation, NED→ENU box conversion, cross-node consistency checks C1–C7 | none |
| `grid.py` | Band grid geometry, query-point generation, chunking, response ingestion | none |
| `frontier.py` | Unknown-region filter, frontier mask, clustering, distance-to-unknown | none (numpy, scipy.ndimage) |
| `planner.py` | Traversability, snapping, Dijkstra, tiered candidates, visibility/gain, utility, commitment, blacklist | none |
| `lifecycle.py` | Goal lifecycle (reached / cleared / progress / timeout / reject) with injected clock | none |
| `fsm.py` | Mission state machine and interlock evaluation with injected clock; emits abstract actions | none |
| `explorer_node.py` | ROS adapter: subscriptions, service clients and servers, timers, action execution | rclpy |

Purity rule (VR-1): modules other than `explorer_node.py` must not import `rclpy` or any `*_msgs`. Enforced by a test.

### 3.2 Execution model
- `rclpy` single-threaded executor. **No blocking calls inside callbacks** (never `spin_until_future_complete` from a callback; that deadlocks the executor).
- **Cycle timer** (`cycle_period_s`, default 2.0 s): runs one planning cycle. QueryMap requests go out with `call_async`; the cycle finishes in the callback of the last chunk's response. If a cycle is still in flight when the timer fires again, the tick is skipped and counted (`status.skipped_cycles`).
- **Monitor timer** (10 Hz, fixed): interlock evaluation (§9.3), goal-lifecycle checks (§8), heartbeat bookkeeping.
- **Status timer** (1 Hz, fixed): publishes `state` and `status`.
- The 2.0 s cycle matches the map: 2 Hz insertion (`insert_period_s: 0.5`), EDT every 2 s (`edt_period_s: 2.0`).

## 4. Interfaces (IR-1)

`<ns>` = parameter `as2_namespace` (default `drone0`). QoS "R10" = RELIABLE, KEEP_LAST 10. "SensorData" = BEST_EFFORT, KEEP_LAST 5.

### 4.1 Subscriptions
| Topic | Type | QoS | Publisher / rate | Use |
|---|---|---|---|---|
| `/<ns>/self_localization/pose` | `geometry_msgs/PoseStamped` | SensorData | AS2 `raw_odometry`, frame `earth` | Robot pose (PX4-fused estimate, one estimator of record) |
| `/<ns>/platform/info` | `as2_msgs/PlatformInfo` | R10 | AS2 platform | `connected`, `armed`, `offboard`, `status.state` (`FLYING = 3`) |
| `/lio_health/state` | `std_msgs/UInt8` | SensorData (compatible with the R10 publisher) | `lio_health_guard`, 1 Hz | `OK = 0`, `DEGRADED = 1`; silence = unknown |
| `/geofence/state` | `std_msgs/UInt8` | SensorData | `geofence_watchdog`, 2 Hz | `OK = 0`, `SOFT = 1`, `HARD = 2`; silence = watchdog death |
| `/go_to_with_avoidance/alert` | `std_msgs/String` | R10 | `planner_shim`, on events | Detect `goal REJECTED…` (OI-3) |
| `/Odometry` | `nav_msgs/Odometry` | SensorData | FAST-LIO, ~10 Hz | **Monitor only** (SR-9). Never used for goal computation. |

### 4.2 Publications
| Topic | Type | QoS | Rate | Content |
|---|---|---|---|---|
| `/go_to_with_avoidance/goal` | `geometry_msgs/PoseStamped` | R10 | event-driven, ≤ 1 per selection + ≤ 1 re-publish (SR-6) | The only motion-related output |
| `/go_to_with_avoidance/hover_request` | `std_msgs/Empty` | R10 | on interlock entry per §9.3 | Shared hover bus also used by the shim. Consumer [VERIFY at Gate 6] (OI-4) |
| `/frontier_explorer/state` | `std_msgs/UInt8` | R10 | 1 Hz heartbeat | State code (§9.1) |
| `/frontier_explorer/status` | `std_msgs/String` | R10 | 1 Hz | JSON, schema §4.5 |
| `/frontier_explorer/alert` | `std_msgs/String` | R10 | on every transition | `"<STATE>: <reason>"` |
| `/frontier_explorer/markers` | `visualization_msgs/MarkerArray` | R10 | per cycle, only if `publish_markers` | Frontier cells, clusters, candidates, goal, blacklist. Off-board viewing only (no RViz on the vehicle). Default **false**. |

No other publisher may exist in the node (SR-2, T-N10).

### 4.3 Services provided
All `std_srvs/srv/Trigger`. `success=false` with a human-readable `message` whenever the request is refused.

| Service | Accepted in states | Effect |
|---|---|---|
| `/frontier_explorer/start` | `IDLE`, `DONE` | New mission after preconditions P1–P7 (§9.4) |
| `/frontier_explorer/pause` | `WAIT_MAP`, `SELECTING`, `NAVIGATING` | → `PAUSED(OPERATOR)` |
| `/frontier_explorer/resume` | `PAUSED` | → `SELECTING` if all interlocks are clear and C1–C7 pass |
| `/frontier_explorer/stop` | all except `FAULT` | → `IDLE` |

### 4.4 Service clients
| Service | Type | When |
|---|---|---|
| `/map_interface/query` | `map_interface/srv/QueryMap` | Every cycle |
| `<shim_node>/get_parameters` | `rcl_interfaces/srv/GetParameters` | Node start and START (C2) |
| `<map_node>/get_parameters` | same | Node start and START (C3) |
| `<fence_node>/get_parameters` | same | Node start and START (C4) |

Default node names: `shim_node = /go_to_with_avoidance`, `map_node = /map_interface`, `fence_node = /geofence_watchdog`. All are parameters.

### 4.5 Status JSON schema
```json
{
  "state": "NAVIGATING", "reason": "", "mission_elapsed_s": 41.2,
  "goal": {"x": 1.3, "y": -0.7, "z": 1.3, "age_s": 6.0, "utility": 2.4, "tight": false, "republished": false},
  "clusters": 3, "frontier_cells": 57, "known_free_m2": 18.4,
  "blacklist": 2, "empty_cycles": 0, "skipped_cycles": 0,
  "last_query_ms": 84.0, "last_plan_ms": 212.0,
  "interlocks": [], "unexplored_clusters": []
}
```
`unexplored_clusters` is filled at `DONE`: `[{ "cells": n, "centroid": [x, y] }]` for clusters left with no valid candidate. `goal.tight` is true when the goal came from the tight tier (§7.12).

### 4.6 Messages
IR-2: standard message types only, so no rosidl package and the package stays `ament_python`.

## 5. Parameters

All parameters are read once at node start. Changing them requires a node restart (no dynamic reconfigure). C1 validates every range; a violation is a `FAULT(CONFIG)`.

### 5.1 Box, grid and band
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `x_min`, `x_max` | double | −4.5, 4.5 | m | min < max | Planning box, ENU. **Must equal the shim's** (C2) |
| `y_min`, `y_max` | double | −4.5, 4.5 | m | min < max | same |
| `z_min`, `z_max` | double | 0.2, 2.3 | m | min < max | same |
| `obstacle_inflation` | double | 0.4 | m | ≥ 0, equal to shim (C2) | Box-face margin passed to `check_goal` |
| `resolution` | double | 0.2 | m | > 0, equal to map (C3) | Grid cell size |
| `z_fly` | double | 1.3 | m | layer centre in `[z_min+infl, z_max−infl]` (C5) | Flight altitude. Snapped to the layer containing it (§7.2) |
| `band_half_layers` | int | 0 | layers | **must be 0 in Phase 1** | Reserved for Phase 2 |
| `max_grid_cells` | int | 40000 | cells | ≥ 1 | Compute bound (C6) |

### 5.2 Map sampling
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `cycle_period_s` | double | 2.0 | s | [0.5, 10] | Planning cycle period |
| `max_points_per_request` | int | 4096 | points | [256, 65536] | Chunk size per QueryMap call |
| `query_timeout_s` | double | 1.0 | s | (0, `cycle_period_s`) | Per-chunk timeout |
| `map_fail_max` | int | 3 | cycles | ≥ 1 | Consecutive failed cycles ⇒ interlock I5 |
| `map_ready_min_free_cells` | int | 50 | cells | ≥ 1 | FREE cells required to leave `WAIT_MAP` |
| `wait_map_timeout_s` | double | 30.0 | s | > 0 | `WAIT_MAP` limit ⇒ I5 |

### 5.3 Frontier extraction
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `min_unknown_region_cells` | int | 9 | cells | ≥ 1 | Unknown 4-connected regions smaller than this are speckle (§7.5) |
| `min_cluster_cells` | int | 3 | cells | ≥ 1 | Smallest frontier cluster kept |
| `max_clusters` | int | 32 | clusters | ≥ 1 | Keep the largest; ties by label order |

### 5.4 Traversability and viewpoints
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `traverse_clearance_m` | double | 0.4 | m | ≥ 0; recommended = EGO `obstacle_inflation` | Cells usable for the reachability search; also the tight-tier clearance and standoff |
| `goal_clearance_m` | double | 0.6 | m | ≥ `traverse_clearance_m` | Preferred-tier minimum clearance at a goal |
| `unknown_standoff_m` | double | 0.6 | m | ≥ `traverse_clearance_m` | Preferred-tier minimum distance from a goal to effective unknown space |
| `allow_tight_goals` | bool | true | — | — | Enable the tight tier (§7.12). With false, rooms beyond corridors narrower than ~2·`goal_clearance_m` cannot be explored (S-06b) |
| `view_range_m` | double | 3.0 | m | (0, map `max_insert_range`] (C3) | Max goal-to-frontier distance counted as visible |
| `candidate_stride_cells` | int | 2 | cells | ≥ 1 | Candidate subsampling (0.4 m at defaults) |
| `max_candidates_per_cluster` | int | 256 | cells | ≥ 1 | Compute bound; keep lowest path distance, ties by `(i, j)` |
| `max_gain_samples` | int | 64 | cells | ≥ 1 | Frontier cells sampled per gain evaluation |
| `start_snap_radius_m` | double | 0.6 | m | ≥ 0 | Search radius when the robot cell is not traversable |

### 5.5 Selection
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `distance_weight` | double | 1.0 | 1 | ≥ 0 | Utility = visible frontier length (m) − weight × path length (m) |
| `switch_margin` | double | 2.0 | m | ≥ 0 | Commitment hysteresis |

### 5.6 Goal lifecycle
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `reach_tolerance_xy_m` | double | 0.3 | m | (0, `visited_radius_m`) | Arrival radius |
| `reach_tolerance_z_m` | double | 0.3 | m | > 0 | Arrival height tolerance |
| `nominal_speed_m_s` | double | 0.5 | m/s | (0, 1.0] | For the timeout estimate (EGO `max_vel` 1.0) |
| `goal_timeout_factor` | double | 3.0 | 1 | ≥ 1 | Timeout = max(min, factor × path / speed) |
| `goal_timeout_min_s` | double | 20.0 | s | > 0 | Timeout floor |
| `progress_window_s` | double | 15.0 | s | > `cycle_period_s` | No-progress window |
| `progress_min_m` | double | 0.3 | m | > 0 | Required path-distance reduction per window |
| `start_motion_timeout_s` | double | 5.0 | s | > 0 | No motion after publish ⇒ one re-publish |
| `reject_window_s` | double | 1.0 | s | > 0 | Shim alerts within this window are attributed to the last goal |
| `blacklist_radius_m` | double | 1.0 | m | > 0 | Exclusion radius for failed goals |
| `visited_radius_m` | double | 0.6 | m | > `reach_tolerance_xy_m` | Exclusion radius for reached viewpoints (must cover the arrival point) |
| `blacklist_max_entries` | int | 64 | entries | ≥ 1 | FIFO capacity |
| `blacklist_ttl_s` | double | 0.0 | s | ≥ 0 (0 = whole mission) | Entry lifetime |

### 5.7 Termination
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `done_confirm_cycles` | int | 3 | cycles | ≥ 1 | Consecutive empty cycles ⇒ `DONE` |
| `max_mission_s` | double | 900.0 | s | > 0 | Mission wall-clock limit (includes `PAUSED` time) ⇒ `DONE(TIMEOUT)` |
| `on_done` | string | `hold` | — | `hold` \| `return_start` | Final goal policy |

### 5.8 Interlocks
| Name | Type | Default | Unit | Valid | Purpose |
|---|---|---|---|---|---|
| `pose_timeout_s` | double | 0.5 | s | > 0 | I1 |
| `platform_info_timeout_s` | double | 2.0 | s | > 0 | I2, P2 |
| `lio_heartbeat_timeout_s` | double | 3.0 | s | > 1.0 (publisher 1 Hz) | I3 |
| `fence_heartbeat_timeout_s` | double | 1.5 | s | > 0.5 (publisher 2 Hz) | I4 |
| `frame_divergence_max_m` | double | 0.5 | m | > 0 | I6 |
| `frame_divergence_persist_s` | double | 1.0 | s | ≥ 0 | I6 persistence |
| `frame_check_max_dt_s` | double | 0.2 | s | > 0 | Max stamp gap for an AS2/LIO sample pair |
| `remote_param_timeout_s` | double | 2.0 | s | > 0 | C7 |

### 5.9 Interfaces and diagnostics
| Name | Type | Default | Purpose |
|---|---|---|---|
| `as2_namespace` | string | `drone0` | AS2 topic namespace |
| `goal_frame_id` | string | `odom` | `header.frame_id` of published goals |
| `shim_node` | string | `/go_to_with_avoidance` | C2 target |
| `map_node` | string | `/map_interface` | C3 target |
| `fence_node` | string | `/geofence_watchdog` | C4 target |
| `publish_markers` | bool | false | Visualization output |
| `max_plan_time_s` | double | 1.0 | Warn if a planning cycle exceeds this (PR-1) |

## 6. Configuration consistency checks (SR-1)

Run at node start and again on every START and RESUME. Any failure puts the node in `FAULT(CONFIG)` (at node start) or refuses the request (at START/RESUME), with the failing check and values in the alert and service message. The node keeps publishing `state`/`status` in `FAULT`, so the operator can see why.

| ID | Check | Source of truth |
|---|---|---|
| C1 | Every parameter within its valid range (§5); `band_half_layers == 0` | local |
| C2 | Shim `x_min … z_max` and `obstacle_inflation` equal local values within 1e-6 | `<shim_node>/get_parameters` |
| C3 | Map `map_backend == cpu_grid` (DP-1; the server also refuses other backends, so this is defence in depth); map `resolution` equals local; `B` ⊆ map bounds shrunk by `res` on every face; `view_range_m ≤ max_insert_range` | `<map_node>/get_parameters` |
| C4 | `B` ⊆ ENU(geofence box) shrunk by `soft_margin` on every face | `<fence_node>/get_parameters`, NED→ENU per §2.1 |
| C5 | Layer centre `z_L` (§7.2) ∈ `[z_min + infl, z_max − infl]` | local |
| C6 | `nx · ny ≤ max_grid_cells` | local |
| C7 | All three remote parameter services respond within `remote_param_timeout_s` | remote |

**Known finding (OI-2), verified in §18:** with today's branch defaults, C4 fails. The shim box is ENU x,y ∈ [−4.5, 4.5], z ∈ [0.2, 2.3]; the watchdog box (NED x,y ±1.5, z [−2.5, 0.3], sized for the Gate 4 tether) becomes ENU x,y ∈ [−1.5, 1.5], z ∈ [−0.3, 2.5], so with `soft_margin` 0.5 the box must fit inside x,y [−1.0, 1.0], z [0.2, 2.0]. This refusal is correct behaviour. The three configs must be sized together for the real venue before Gate 7.

## 7. Algorithm (Phase 1)

### 7.1 Cycle overview
```
cycle():
  if cycle_in_flight: skipped_cycles += 1; return
  pts = grid.query_points()                         # §7.2
  send chunks async (§7.3); on all responses → plan(occ, clr)
plan(occ, clr):
  t0 = monotonic()
  unk_eff  = unknown_region_filter(occ == UNKNOWN)  # §7.5
  front    = (occ == FREE) & dilate4(unk_eff)       # §7.6
  clusters = label8(front), size filter             # §7.7
  trav     = (occ == FREE) & (clr >= traverse_clearance_m)   # §7.8
  d_unk    = edt(~unk_eff) * res                    # §7.9
  start    = snap(robot_pose)                       # §7.10
  dist     = dijkstra(trav, start)                  # §7.11
  per cluster: tiered candidates, best viewpoint    # §7.12
  decision = select_with_commitment(...)            # §7.13–§7.14
  last_plan_ms = (monotonic() − t0) · 1e3; warn if > max_plan_time_s
  hand decision to fsm
```

### 7.2 Grid geometry
Octomap voxel `k` spans `[k·res, (k+1)·res)`, centre `(k + 0.5)·res`. Sampling at voxel **centres** avoids boundary ambiguity (the same reason `wp-c`'s synthetic scene places surfaces mid-voxel).

```
kx_min = ceil (x_min / res − 0.5 − EPS)     kx_max = floor(x_max / res − 0.5 + EPS)
ky_min = ceil (y_min / res − 0.5 − EPS)     ky_max = floor(y_max / res − 0.5 + EPS)
nx = kx_max − kx_min + 1                    ny = ky_max − ky_min + 1
cell (i, j), 0 ≤ i < nx, 0 ≤ j < ny  ↦  x = (kx_min + i + 0.5)·res,  y = (ky_min + j + 0.5)·res
k_z = floor(z_fly / res + EPS)              z_L = (k_z + 0.5)·res
```
- At defaults: `nx = ny = 46` (centres from −4.5 to 4.5 inclusive, because ±4.5 are voxel centres), 2116 cells, `z_L = 1.3`. A face-aligned box such as ±1.5 at 0.1 m gives 30 centres from −1.45 to 1.45.
- Arrays are indexed `[i, j]` (x first). Row-major point order: `i` outer, `j` inner.
- `EPS` is required. Without it, `1.2 / 0.2 = 5.999999999999999` gives `k_z = 5` (the 1.1 m layer instead of 1.3 m), and box edges such as `(−4.85, 5.85)` at 0.1 m give wrong index ranges. Verification (§18) found naive float arithmetic disagreeing with exact decimal arithmetic in 113 of the tested 5 cm edge/layer cases and the `EPS` form in none. T-G02 guards this.
- World→cell for the robot: `i = floor(x / res + EPS) − kx_min`, `j = floor(y / res + EPS) − ky_min`, clamped to the grid (the clamp is recorded; a pose outside `B` is interlock territory for the watchdog and shim).

### 7.3 Map sampling
- Points are generated once at start (the geometry is static) and split into consecutive chunks of at most `max_points_per_request`.
- All chunks of a cycle go out immediately with `call_async`. Each carries a deadline `now + query_timeout_s`, checked by the monitor timer.
- Per-response validation: `len(occupancy) == len(clearance) == len(request.points)`; every occupancy value ∈ {0, 1, 2}. Clearance values below 0 other than −1.0 are invalid.
- A cycle **fails** if any chunk times out or fails validation. Its partial data is discarded, never merged with an older cycle. `map_fail_count` increments on failure and resets on success. At `map_fail_count ≥ map_fail_max`: interlock I5.
- `last_query_ms` = time from first send to last response.

### 7.4 Classification arrays
`occ` (`uint8`, shape `(nx, ny)`), `clr` (`float32`). Masks: `FREE = occ == 0`, `OCC = occ == 1`, `UNK = occ == 2`. Any cell with `clr < 0` is treated as not traversable and not a candidate, regardless of `occ`.

### 7.5 Unknown-region filter
LiDAR raycasting at 0.2 m leaves isolated unknown voxels inside observed free space. Each one would otherwise produce up to 4 frontier cells.
```
labels = label(UNK, structure = 4-connectivity)
unk_eff = UNK & (size(labels) ≥ min_unknown_region_cells)
```
Real openings (a doorway into an unseen room, a corridor mouth) lead into large unknown regions and survive. Small unknown pockets are treated as non-frontier-generating **and** non-blocking for line of sight. They are **not** treated as traversable (§7.8 uses `FREE` only).

### 7.6 Frontier mask (SR-8)
```
front[c] = FREE[c] ∧ ∃ n ∈ N4(c): unk_eff[n]
```
Neighbours outside `G` do not exist (dilation with border value 0). Unknown space beyond the planning box (including the `occupancy = UNKNOWN` returned outside map bounds) therefore never creates frontiers. Only cells in the band layer take part, so floor and ceiling voxels the Mid-360 (−7°…+52° vertical FOV) never observes cannot keep a mission alive forever. A free cell touching unknown only diagonally is not a frontier.

### 7.7 Clusters
```
labels = label(front, structure = 8-connectivity)
keep clusters with size ≥ min_cluster_cells
if count > max_clusters: keep the max_clusters largest (ties: lower label first)
```
Label order is `scipy.ndimage.label` raster order over `[i, j]`, which is deterministic.

### 7.8 Traversability
```
trav[c] = FREE[c] ∧ clr[c] ≥ traverse_clearance_m
```
- `clr` is a 3D distance, so obstacles just above or below the flight layer (a table top 0.3 m below) correctly reduce traversability.
- Because unknown counts as free in the EDT, `trav` is optimistic next to unknown space.
- **Discretisation:** `clr` is measured between voxel centres, so it overstates the physical gap by up to `res/2` per side. At 0.2 m, a 0.6 m doorway (3 free cells) has centre clearance 0.40 and is grid-traversable at 0.4, while a 0.4 m doorway (2 cells) has 0.20 and is not (S-02). EGO physically needs about 2 × 0.4 = 0.8 m. A grid-traversable but physically impassable gap ends in NO_PROGRESS or GOAL_TIMEOUT and a failed blacklist entry (§8.2).
- The reachability search is an estimate for ranking and filtering goals; **EGO's local map remains the authority on feasibility.**

### 7.9 Distance to effective unknown
```
d_unk = distance_transform_edt(~unk_eff) · res        # metres to nearest unk_eff cell
if not unk_eff.any(): d_unk = +inf everywhere
```

### 7.10 Robot snapping
```
s0 = world_to_cell(pose)
if trav[s0]: start = s0
else: start = trav cell with minimum Euclidean distance to s0 within start_snap_radius_m
      (ties: lowest (i, j))
if none: STUCK (interlock I9)
```

### 7.11 Reachability (Dijkstra)
- Graph: cells with `trav`. 8-connected. Step cost `res` (axial), `res·√2` (diagonal).
- **No corner cutting:** a diagonal step from `(i, j)` to `(i+di, j+dj)` requires `trav[i+di, j]` and `trav[i, j+dj]`.
- Output `dist[c]` in metres (`+inf` if unreachable). Implementation: binary heap (`heapq`); pop ties broken by `(dist, i, j)`.

### 7.12 Viewpoints: tiers, visibility, gain
For each kept cluster `C`:
1. **Region** `R_C`: bounding box of `C` expanded by `ceil(view_range_m / res)` cells, clipped to `G`.
2. **Distance to cluster** `d_C[c]`: Euclidean distance (m) from `c` to the nearest cell of `C`, computed as a distance transform of `¬C` over `R_C`.
3. **Base set** `B_C`: cells `c ∈ R_C` with
   - `i mod candidate_stride_cells == 0` and `j mod candidate_stride_cells == 0`
   - `trav[c]` and `dist[c] < inf` and `d_C[c] ≤ view_range_m`
   - not within any active blacklist entry's radius (§8.5).
4. **Tiers.** Both require `check_goal((x_c, y_c, z_L), B, infl) == (True, '')`, using `planner_shim.shim_core.check_goal`, the same function the shim runs (SR-3).
   - **Preferred** `P_C = {c ∈ B_C : clr[c] ≥ goal_clearance_m ∧ d_unk[c] ≥ unknown_standoff_m}`
   - **Tight** `T_C = {c ∈ B_C : d_unk[c] ≥ traverse_clearance_m}` (clearance ≥ `traverse_clearance_m` is already implied by `trav`)
   - `K_C = P_C` if non-empty; else `T_C` if `allow_tight_goals` and non-empty; else `∅` (cluster exhausted). The tier is recorded (`goal.tight`).
   - Why tiers: a corridor narrower than about 2·`goal_clearance_m` holds no preferred cell, so without the tight tier rooms beyond 0.8 m corridors are never reached (S-06b).
   - If `|K_C| > max_candidates_per_cluster`, keep the lowest `(dist[c], i, j)`.
5. **Samples** `S_C`: if `|C| ≤ max_gain_samples`, all of `C`; otherwise every `ceil(|C| / max_gain_samples)`-th cell in raster order.
6. **Visibility** `vis[k, f]` for `k ∈ K_C`, `f ∈ S_C` (vectorised over all pairs):
   - in range: `‖k − f‖ · res ≤ view_range_m`; out-of-range pairs are not sampled (bounds sample count to `2·view_range_m/res`)
   - `m = ceil(2·‖k − f‖)` (steps of `res/2`, distances in cells); samples at `k + (f − k)·t/m` for `t = 1 … m−1`
   - nearest cell of a sample = `floor(coordinate + 0.5)` per axis
   - **blocked** if any sample's nearest cell is `OCC` or `unk_eff` **and is neither the cell `k` nor the cell `f`**. Without that exclusion, samples next to an endpoint round onto the endpoint cell, so every ray to a wall counts as blocked (verification finding, §18).
7. **Gain** `g(k) = |{f ∈ S_C : vis[k, f]}| · |C| / |S_C|`.
8. **Best viewpoint** `k*_C`: maximum `g`; ties → smallest `d_C[k]` (closest to the frontier) → smallest `dist[k]` → smallest `(i, j)`. Clusters with empty `K_C` or `g(k*) = 0` are **exhausted** this cycle.
   Why closest to the frontier: in a corridor every candidate sees the same cross-section, so gains tie. Breaking ties by path distance made the explorer creep (13 goals along an 8.4 m corridor); breaking by frontier proximity gives about 2.4 m steps (4 goals) (S-03, §18).

### 7.13 Utility
```
U(C) = g(k*_C)·res − distance_weight · dist[k*_C]
```
Visible frontier length and path length are both in metres. At `distance_weight = 1.0`, one extra metre of travel must buy one extra metre of visible frontier. As the weight grows, behaviour tends to classic nearest-frontier (Yamauchi).

### 7.14 Selection with commitment (FR-6)
```
best = argmax_C U(C)     ties: larger |C|, then lower cluster label
if state == NAVIGATING:
    U_cur = max over kept clusters C with g ∈ K_C (computed WITHOUT the
            max_candidates_per_cluster cap, same tier rule) of  g_C(g)·res − distance_weight·dist[g]
    if U_cur exists:
        switch only if U(best) > U_cur + switch_margin
    else:                                                   # g is no longer a candidate of any cluster
        if no cell of V_g is still front: FRONTIER_CLEARED (§8.2)
        else: re-select (goal invalidated by map change, not blacklisted)
decision = goal(k*_best) or NONE (no non-exhausted cluster)
```
An exact tie between two clusters resolves to the lower label, which is the lower raster position (S-07).

### 7.15 Goal message
- `header.frame_id = goal_frame_id`, `header.stamp = now`.
- `position = (x_k, y_k, z_L)`.
- `orientation` = current vehicle yaw with roll and pitch removed. The Mid-360 is 360° horizontally, so yaw does not affect coverage; keeping it avoids needless rotation. EGO/AS2 yaw handling is [VERIFY at Gate 6] (OI-5).

### 7.16 Determinism (FR-13)
No randomness, no wall-clock-dependent ordering. Given identical `occ`, `clr`, pose, blacklist and state, the decision is bit-identical (T-P03).

### 7.17 Complexity and measured cost (N = nx·ny cells)
| Step | Cost |
|---|---|
| Labelling ×2, dilation, EDTs | O(N) (scipy C implementations) |
| Dijkstra | O(N log N), pure Python |
| Candidates and visibility | ≤ `max_clusters` × `max_candidates_per_cluster` × `max_gain_samples` × (2·`view_range_m`/`res`) sampled points, vectorised |

Measured prototype cost is in §11. The candidate filter and visibility **must be vectorised** (numpy); the loop form measured 940 ms at N = 10,000 with 32 clusters, the vectorised form 205 ms.

### 7.18 Termination (FR-10)
- `empty_cycles` increments on every successful cycle whose decision is NONE and resets on any goal decision.
- `empty_cycles ≥ done_confirm_cycles` ⇒ `DONE(COMPLETE)` with `unexplored_clusters` = clusters that exist but are exhausted.
- Mission elapsed (wall clock since T1, including `PAUSED` time) > `max_mission_s` ⇒ `DONE(TIMEOUT)`, evaluated in active states only; a mission resumed past its limit ends at the next monitor tick.
- `DONE` action: `on_done = hold` ⇒ hold goal (§8.6); `return_start` ⇒ goal at the recorded start pose if it passes `check_goal` and the §8.6 publication conditions, otherwise hold.

## 8. Goal lifecycle (FR-8)

Evaluated by the monitor timer (10 Hz) and at every cycle.

### 8.1 On publish
Record `goal`, `t_pub`, `pose_at_pub`, `path_len = dist[goal]`,
`timeout = max(goal_timeout_min_s, goal_timeout_factor · path_len / nominal_speed_m_s)`,
`best_dist = path_len`, `t_best = t_pub`, `republished = false`,
and the set `V_g` of sample frontier cells visible from the goal.

### 8.2 Events
| Event | Condition | Result |
|---|---|---|
| `REACHED` | `‖pose_xy − goal_xy‖ ≤ reach_tolerance_xy_m` ∧ `|pose_z − goal_z| ≤ reach_tolerance_z_m` | Add **visited** blacklist entry (radius `visited_radius_m`) → `SELECTING` |
| `FRONTIER_CLEARED` | At a cycle: no cell of `V_g` is still `front` | → `SELECTING` (no blacklist) |
| `NO_PROGRESS` | At a cycle: `dist_now[goal] < best_dist − progress_min_m` resets `best_dist, t_best`; otherwise if `now − t_best > progress_window_s` | **Failed** blacklist entry (radius `blacklist_radius_m`) → `SELECTING` |
| `GOAL_TIMEOUT` | `now − t_pub > timeout` | Failed blacklist entry → `SELECTING` |
| `NO_START` | `now − t_pub > start_motion_timeout_s` ∧ `‖pose − pose_at_pub‖ < progress_min_m` ∧ `¬republished` | Re-publish the same goal once; `republished = true` |
| `REJECTED_LIO` | Shim alert starting with `goal REJECTED: LIO degraded` within `reject_window_s` of a publish | → `PAUSED(I3)` |
| `REJECTED_OTHER` | Any other shim alert starting with `goal REJECTED` within `reject_window_s` of a publish | → `FAULT(I7)`: a pre-validated goal was refused, so configs have drifted |
| `BETTER_GOAL` | §7.14 switch rule | Publish new goal, reinitialise §8.1 |

`dist_now` comes from that cycle's Dijkstra from the current robot cell. If the goal cell is not reachable this cycle, `dist_now[goal] = inf` and no progress is credited.

### 8.3 Why a visited blacklist
After `REACHED`, a frontier occluded from that viewpoint (beyond a door too narrow to traverse) would otherwise re-select the same viewpoint forever. Blacklisting the visited viewpoint forces a different viewpoint or marks the cluster exhausted (S-02). `visited_radius_m > reach_tolerance_xy_m` guarantees the entry covers the arrival point.

### 8.4 Shim alert matching
Prefix match on `std_msgs/String.data`. This depends on free text in `go_to_with_avoidance_node.py` (`'goal REJECTED: LIO degraded'` and `f'goal REJECTED: {why}'`). T-N06 pins it; OI-3 proposes a structured status.

### 8.5 Blacklist
- Entry `(x, y, radius, reason ∈ {VISITED, FAILED}, t_added)`. Capacity `blacklist_max_entries`, FIFO eviction.
- Active if `blacklist_ttl_s == 0` or `now − t_added ≤ blacklist_ttl_s`.
- A candidate cell is excluded if its centre lies within `radius` of any active entry (2D distance).
- Cleared on START (new mission). Kept across PAUSE/RESUME.

### 8.6 Hold goal
`hold = (clamp(pose_x, x_min+infl+0.05, x_max−infl−0.05), clamp(pose_y, …), clamp(pose_z, z_min+infl+0.05, z_max−infl−0.05))`, yaw per §7.15.

Published only if **all** of these hold; otherwise it is skipped and the alert says why:
- the platform is `FLYING` (SR-4);
- the pose is fresh;
- `check_goal(hold)` passes;
- **neither I3 nor I4 is raised** (SR-10). This applies to every transition that publishes a hold goal (T17, T20, `on_done`), so an operator `stop` during a LIO-degradation or geofence response never competes with the guard or the watchdog.

The same four conditions apply to the `return_start` goal.

## 9. Mission state machine

### 9.1 States
| Code | State | Meaning |
|---|---|---|
| 0 | `IDLE` | No mission. Never publishes goals. |
| 1 | `WAIT_MAP` | Mission started; waiting for enough known free space |
| 2 | `SELECTING` | Choosing the next goal; stays here while cycles return NONE (T6) until a goal (T5) or `DONE` (T7) |
| 3 | `NAVIGATING` | A goal is active |
| 4 | `PAUSED` | Latched stop with a reason. Only `resume` or `stop` leave it. |
| 5 | `DONE` | Mission ended (`COMPLETE` or `TIMEOUT`) |
| 6 | `FAULT` | Unrecoverable (config drift, internal error). **Node restart required.** |

Active states: `WAIT_MAP`, `SELECTING`, `NAVIGATING`.

### 9.2 Transitions
| # | From | Event / guard | To | Actions |
|---|---|---|---|---|
| T1 | `IDLE`, `DONE` | `start` ∧ P1–P7 pass | `WAIT_MAP` | Reset blacklist, counters, timers; record start pose; alert |
| T2 | `IDLE`, `DONE` | `start` ∧ any P fails | same | `success=false`, message lists the failures |
| T3 | `WAIT_MAP` | cycle ok ∧ `count(FREE) ≥ map_ready_min_free_cells` | `SELECTING` | — |
| T4 | `WAIT_MAP` | in state > `wait_map_timeout_s` | `PAUSED` | Reason `MAP_NOT_READY`; per I5 row |
| T5 | `SELECTING` | decision = goal | `NAVIGATING` | Publish goal (§8.1) |
| T6 | `SELECTING` | decision = NONE | `SELECTING` | `empty_cycles += 1` |
| T7 | `SELECTING` | `empty_cycles ≥ done_confirm_cycles` | `DONE` | Reason `COMPLETE`; `on_done` action; alert with summary |
| T8 | `NAVIGATING` | `REACHED` | `SELECTING` | Visited entry |
| T9 | `NAVIGATING` | `FRONTIER_CLEARED` | `SELECTING` | — |
| T10 | `NAVIGATING` | `NO_PROGRESS` ∨ `GOAL_TIMEOUT` | `SELECTING` | Failed entry |
| T11 | `NAVIGATING` | `NO_START` | `NAVIGATING` | Re-publish once |
| T12 | `NAVIGATING` | `BETTER_GOAL` | `NAVIGATING` | Publish new goal |
| T13 | `NAVIGATING` | `REJECTED_LIO` | `PAUSED` | Reason I3; per I3 row |
| T14 | `NAVIGATING` | `REJECTED_OTHER` | `FAULT` | Reason I7; per I7 row |
| T15 | active | any interlock I1–I9 raised | `PAUSED` or `FAULT` per §9.3 | Per row |
| T16 | active | mission elapsed > `max_mission_s` | `DONE` | Reason `TIMEOUT`; `on_done` action |
| T17 | active | `pause` | `PAUSED` | Reason `OPERATOR`; hold goal (§8.6), hover request |
| T18 | `PAUSED` | `resume` ∧ all interlocks clear ∧ C1–C7 pass | `SELECTING` | Alert; blacklist kept; goal timers discarded |
| T19 | `PAUSED` | `resume` ∧ not clear | `PAUSED` | `success=false` with active interlocks |
| T20 | all except `FAULT` | `stop` | `IDLE` | Hold goal (§8.6) |
| T21 | any | unhandled exception in planning or callbacks | `FAULT` | Reason I8; per I8 row |

QueryMap cycles run only in `WAIT_MAP`, `SELECTING` and `NAVIGATING`. Every transition, including self-transitions T6, T11 and T12, publishes an alert only when the state or reason changes (T6 does not alert).

### 9.3 Interlocks (SR-5, SR-10)
Interlock **conditions** are evaluated at 10 Hz in every state, so `status.interlocks` is always current and T18/T19 can decide. Interlock **transitions** (T15) fire only from active states. Within one monitor tick, order of evaluation is: T21 → interlocks (T15) → mission timeout (T16) → goal lifecycle (T8–T14). If several interlocks are raised in the same tick, **FAULT beats PAUSED**, and among PAUSED reasons the lowest ID is recorded as `reason` while all go in `status.interlocks`.

| ID | Condition | Target | Hold goal | Hover request | Rationale |
|---|---|---|---|---|---|
| I1 | No pose for > `pose_timeout_s` | `PAUSED` | no | yes | Pose unknown, so no safe hold can be computed |
| I2 | `platform/info`: `status.state ≠ FLYING` ∨ `¬connected` ∨ `¬armed` ∨ `¬offboard`, or no message for > `platform_info_timeout_s` | `PAUSED` | no | no | AS2 or the pilot owns the vehicle (SR-4) |
| I3 | `lio_health/state == DEGRADED` or no message for > `lio_heartbeat_timeout_s` | `PAUSED` | no | no | `lio_health_guard` and the shim already request hover; don't compete |
| I4 | `geofence/state ≠ OK` or no message for > `fence_heartbeat_timeout_s` | `PAUSED` | no | no | The watchdog owns the response (hover or land). Heartbeat silence = watchdog death ⇒ operator procedure |
| I5 | `map_fail_count ≥ map_fail_max`, or T4 | `PAUSED` | yes | yes | Planning on stale data is unsafe; stop at a validated point |
| I6 | AS2↔LIO position divergence > `frame_divergence_max_m` for > `frame_divergence_persist_s` | `PAUSED` | no | yes | Map frame and control frame disagree (SR-9) |
| I7 | `REJECTED_OTHER` | `FAULT` | no | yes | Config drift |
| I8 | Unhandled exception | `FAULT` | no | yes | Internal state unknown |
| I9 | STUCK: robot not snappable (§7.10) | `PAUSED` | yes | yes | Vehicle is in untraversable space per the map |

"Hold goal: yes" is always subject to §8.6 (FLYING, fresh pose, `check_goal`, I3/I4 not raised).

### 9.4 START preconditions
| ID | Precondition |
|---|---|
| P1 | C1–C7 pass (§6) |
| P2 | Last `platform/info` ≤ `platform_info_timeout_s` old with `connected ∧ armed ∧ offboard ∧ status.state == FLYING` |
| P3 | Pose age ≤ `pose_timeout_s` |
| P4 | `lio_health/state == OK`, age ≤ `lio_heartbeat_timeout_s` |
| P5 | `geofence/state == OK`, age ≤ `fence_heartbeat_timeout_s` |
| P6 | `/map_interface/query` service available |
| P7 | I6 not raised (frame check has at least one valid sample pair within limits) |

### 9.5 Frame consistency monitor (SR-9)
- Keep the last 20 samples of AS2 pose and LIO `/Odometry` positions with stamps.
- For each new AS2 pose, find the LIO sample with minimum `|Δstamp|`. If `|Δstamp| ≤ frame_check_max_dt_s`, compute the 3D position difference.
- I6 raises when the difference exceeds `frame_divergence_max_m` continuously for `frame_divergence_persist_s`; it clears when it drops below.
- Stamps come from `header.stamp` of both messages (Jetson clock for both; see `px4_odom_bridge` timestamp notes).
- `/Odometry` is read **only** here. Goal computation uses the AS2 (PX4-fused) pose, keeping one estimator of record (masterplan §1.2, §7).

## 10. Hazard and failure analysis

| ID | Failure | Effect | Detection | Response | Test |
|---|---|---|---|---|---|
| H-01 | `map_interface` crash or hang | No map data | Chunk timeout | I5 after `map_fail_max` cycles: hold + hover | T-N03 |
| H-02 | EDT update spike on query | Late cycle | `last_query_ms`, `query_timeout_s` | Skip tick; I5 if persistent | B-01 |
| H-03 | Planning box larger than geofence | Goals near fence ⇒ SOFT breaches | C4 | Refuse START / FAULT at start | T-C04 |
| H-04 | Shim and explorer boxes differ | Explorer goals rejected | C2; REJECTED_OTHER | FAULT(I7) | T-C02, T-L07 |
| H-05 | EKF2 EV origin offset vs LIO world | Map-frame goals land in the wrong place | I6 monitor | PAUSED + hover | T-F06 |
| H-06 | Speckle unknown voxels | Spurious frontiers, dithering | Unknown-region filter | Filtered | T-F03, S-05 |
| H-07 | Two similar clusters | Oscillation | Commitment margin | Hysteresis | T-S04, S-09 |
| H-08 | Goal reachable in the grid but not for EGO (incl. §7.8 discretisation) | Vehicle stalls | NO_PROGRESS / GOAL_TIMEOUT | Failed blacklist, re-select | T-L03, T-L04 |
| H-09 | Door narrower than the grid-traversable width | Room unexplorable | Visited blacklist, exhausted cluster | `DONE` with `unexplored_clusters` | S-02 |
| H-10 | Floor or ceiling never observed | Mission never ends | Band restriction | `DONE` reached | S-04, T-F05 |
| H-11 | Glass or transparent obstacles | LiDAR sees through; EGO may fly into glass | **Not detectable by this node** | Venue survey; size the geofence to exclude glazing (OI-9) | — |
| H-12 | Explorer crash | No new goals | Missing `/frontier_explorer/state` heartbeat (operator) | Fail-safe by design: no streaming; last goal is validated and inside the box; EGO stops there | T-N08 |
| H-13 | Goal message lost (DDS) | Vehicle doesn't move | NO_START | One re-publish; then progress rules | T-L05 |
| H-14 | LIO degradation | Wrong map and pose | `/lio_health/state` | I3 PAUSED; guard and shim hover; no explorer goals, even on `stop` | T-N04, T-M09 |
| H-15 | Watchdog dead or responding | No independent fence, or competing commands | Heartbeat silence / state ≠ OK | I4 PAUSED; no explorer goals, even on `stop`; operator procedure | T-N05, T-M09 |
| H-16 | Pilot takes over (offboard lost) | Goals ignored or fighting the pilot | `platform/info.offboard` | I2 PAUSED, no goals | T-N07 |
| H-17 | Optimistic clearance next to unknown | Goal next to an unseen obstacle | `unknown_standoff_m` (preferred tier), `traverse_clearance_m` (tight tier); EGO local map | Standoff; EGO avoids; `goal.tight` reported | T-S02 |
| H-18 | Float index arithmetic at voxel boundaries | Row or layer mis-selected | `EPS` arithmetic | Correct indices | T-G02 |
| H-19 | Corridor narrower than ~2·`goal_clearance_m` | No preferred goals; rooms beyond unreachable | Empty preferred tier | Tight tier (`allow_tight_goals`) | S-06, S-06b |
| H-20 | Implementation publishes outside its contract (e.g. a stray `/fmu` publisher) | Bypasses the single command path | Static scan and graph introspection | Build fails the test | T-U02, T-N10 |

## 11. Performance requirements

Measurement conditions: Jetson Orin Nano 8 GB, `nvpmodel` **MAXN_SUPER**, CPU governor `schedutil` (no `jetson_clocks`), synthetic data (§18).

| ID | Requirement | Measured (prototype / installed server) | Verification |
|---|---|---|---|
| PR-1 | Planning (§7.4–§7.14) ≤ `max_plan_time_s` = 1.0 s per cycle for N ≤ 10,000 cells; 40,000 cells measured and reported | Vectorised prototype, median: N 2,116 → 25–38 ms; N 10,000 → 92 / 157 / 206 ms (1 / 8 / 32 clusters); N 40,000 → 362 / 426 / 645 ms | B-02, T-S07 |
| PR-2 | QueryMap round trip for one cycle ≤ 100 ms at N ≤ 10,000 with a warm EDT; post-insertion latency reported | Median: N 2,116 → 5.1 ms; N 10,000 → 19.6 ms (4,096-point chunks 15.1 ms), post-insertion 22.3 ms; N 40,000 → 70.8 ms | B-01; VD-008 for real data |
| PR-3 | Node CPU ≤ 10% of one core averaged over a mission-like replay | not yet measured | B-03; compute ledger row |
| PR-4 | Node RSS ≤ 150 MB (numpy + scipy loaded) | not yet measured | B-03 |

All count against the masterplan §1.2 ceiling (2.0 cores / 2.5 GB for map + planner + behaviours).

## 12. Package layout

```
src/frontier_explorer/
├── package.xml                      # ament_python
├── setup.py, setup.cfg, resource/frontier_explorer
├── frontier_explorer/
│   ├── __init__.py
│   ├── config.py   grid.py   frontier.py   planner.py   lifecycle.py   fsm.py
│   └── explorer_node.py
└── test/
    ├── test_config.py   test_grid.py   test_frontier.py   test_planner.py
    ├── test_lifecycle.py   test_fsm.py   test_scenarios.py   test_properties.py
    ├── test_purity.py   test_contract.py   test_node_behavior.py
    └── fixtures.py  (programmatic fixtures, Appendix A)
src/drone_bringup/config/exploration/frontier_explorer.yaml
src/drone_bringup/launch/exploration.launch.py       # explorer node only
tools/replay_tests/test_frontier_explorer_smoke.py   # §13.4
```

Global rules carried over from the Gates 1–4 handover: every new file begins with a comment holding its repo-relative path; `colcon build` always runs with `--parallel-workers 2` and `MAKEFLAGS=-j4`; RViz never runs on the vehicle.

`package.xml` dependencies:
- `depend`: `rclpy`, `std_msgs`, `std_srvs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `rcl_interfaces`, `as2_msgs`, `map_interface`, `planner_shim`
- `exec_depend`: `python3-numpy`, `python3-scipy`
- `test_depend`: `python3-pytest`, `python3-hypothesis`, `ament_flake8`, `ament_pep257`

`planner_shim` is imported for `shim_core.EnuBox` and `check_goal`, keeping one source of truth for goal admissibility. Entry point: `explorer_node = frontier_explorer.explorer_node:main`.

**Install rule:** `drone_bringup/setup.py` installs only top-level `config/*` files plus explicitly listed subdirectories. Add a `data_files` entry for `config/exploration/*`. On `wp-d-planner` the same omission means `config/planning/ego_v2.yaml` is never installed (OI-11); fix it on `wp-d-planner`.

## 13. Test plan

All unit tests run with `colcon test --packages-select frontier_explorer`. Node tests need a DDS domain; in the sandboxed dev shell use `ROS_LOCALHOST_ONLY=1` with `CYCLONEDDS_URI` unset (the same condition WP-E's node tests needed).

### 13.1 Unit tests
| ID | Module | Test | Covers |
|---|---|---|---|
| T-C01 | config | Every range violation in §5 is rejected with its name | SR-1 |
| T-C02 | config | Shim parameter mismatch (each of 7 params) fails C2 | SR-1 |
| T-C03 | config | `map_backend ≠ cpu_grid`; box not inside map bounds shrunk by `res`; resolution mismatch; `view_range_m > max_insert_range` | SR-1 |
| T-C04 | config | NED→ENU fence conversion (asymmetric box) and C4 containment with `soft_margin`; today's defaults fail | SR-1 |
| T-C05 | config | Layer centre outside `[z_min+infl, z_max−infl]` fails C5 | SR-1 |
| T-C06 | config | `band_half_layers ≠ 0` rejected | SR-1 |
| T-G01 | grid | Default box gives 46×46, `z_L = 1.3`, first/last centres ±4.5; face-aligned ±1.5 at 0.1 gives 30 | FR-1 |
| T-G02 | grid | Indices equal exact rational arithmetic for all 5 cm box edges in [−5, 5] at 0.05/0.1/0.2/0.3 m; `z_fly = 1.2` selects the 1.3 m layer | FR-1 |
| T-G03 | grid | Chunking: sizes, order, reassembly equals the original order | FR-1 |
| T-G04 | grid | Response validation: length mismatch, bad occupancy code, bad clearance ⇒ cycle failure | FR-1 |
| T-F01 | frontier | Room with doorway: exactly one cluster, entirely beyond the doorway | FR-2, FR-3 |
| T-F02 | frontier | Unknown beyond the grid edge creates no frontier | SR-8 |
| T-F03 | frontier | Unknown regions smaller than `min_unknown_region_cells` are ignored; exactly at threshold kept | FR-2 |
| T-F04 | frontier | Diagonal-only unknown contact is not a frontier; 8-connected clustering | FR-2, FR-3 |
| T-F05 | frontier | Fully known room: no frontier | FR-10 |
| T-F06 | fsm | Frame monitor: divergence raise after persistence, clear below, stamp gap ignored | SR-9 |
| T-S01 | planner | Dijkstra distances on known grids; no corner cutting through diagonal gaps | FR-4 |
| T-S02 | planner | Tier rules: preferred needs `goal_clearance_m` + `unknown_standoff_m`; tight only when preferred is empty and allowed; stride, `check_goal`, blacklist, view range | FR-5, SR-3 |
| T-S03 | planner | Visibility: blocked by `OCC` and `unk_eff`, not by speckle; endpoint cells never block (a wall cell itself is visible) | FR-5 |
| T-S04 | planner | Commitment: no switch within `switch_margin`; switch beyond it | FR-6 |
| T-S05 | planner | Snapping: within radius picks nearest (tie `(i, j)`); none ⇒ STUCK | FR-4 |
| T-S06 | planner | Utility ordering and cluster tie-breaks (larger, then lower label) | FR-6, FR-13 |
| T-S07 | planner | `max_clusters`, `max_candidates_per_cluster`, `max_gain_samples` caps applied deterministically | FR-3, PR-1 |
| T-S08 | planner | Viewpoint tie-break: equal gain ⇒ closest to cluster before path distance | FR-5 |
| T-S09 | planner | `U_cur` of the just-selected goal equals its `U` (§7.14 definition) | FR-6 |
| T-L01 | lifecycle | REACHED adds a visited entry covering the arrival point | FR-8 |
| T-L02 | lifecycle | FRONTIER_CLEARED when `V_g` leaves `front` | FR-8 |
| T-L03 | lifecycle | NO_PROGRESS after the window without path-distance reduction; progress resets the window | FR-8 |
| T-L04 | lifecycle | GOAL_TIMEOUT formula and floor | FR-8 |
| T-L05 | lifecycle | NO_START re-publishes exactly once | FR-8, SR-6 |
| T-L06 | lifecycle | Blacklist FIFO, TTL, radius, cleared on START | FR-9 |
| T-L07 | lifecycle | Alert matching: LIO rejection vs other rejection vs unrelated alert vs outside window | FR-8 |
| T-M01 | fsm | Every transition T1–T21 fires from its source state with its guard; alerts emitted per §9.2 | FR-11, FR-12 |
| T-M02 | fsm | Every interlock I1–I9: target state, hold and hover actions match §9.3 | SR-5, SR-10 |
| T-M03 | fsm | FAULT beats PAUSED; lowest-ID reason recorded; all listed; tick evaluation order T21 → T15 → T16 → lifecycle | SR-5 |
| T-M04 | fsm | No goal action is ever emitted unless the platform is FLYING | SR-4 |
| T-M05 | fsm | Interlock conditions tracked in `PAUSED`; `resume` refused while any is raised, accepted when clear | SR-5, FR-11 |
| T-M06 | fsm | FAULT is terminal | SR-5 |
| T-M07 | fsm | START refused for each failing P1–P7 | SR-7, FR-11 |
| T-M08 | fsm | `DONE` on `done_confirm_cycles`; timeout (including a resume past the limit); `on_done` variants | FR-10 |
| T-M09 | fsm | `stop`, `pause` and `on_done` while I3 or I4 is raised publish no hold goal | SR-10 |
| T-U01 | purity | Core modules import no `rclpy` / `*_msgs` | VR-1 |
| T-U02 | contract | Static scan of the package source: no `/fmu` string, no `px4_msgs` import, no arming/takeoff/land/mode service or action names | SR-2 |

### 13.2 Scenario tests (Appendix A fixtures, full planning pipeline, no ROS)
Scenarios with a "reveal" use an open-loop script: the fixture holds the true band map; the robot's cell is revealed at start and after each goal (teleport = REACHED), using octomap-like raycasting (every cell within `view_range_m` is a ray target; cells along the ray become known up to and including the first occupied cell). **It is not a simulator** and makes no claim about vehicle behaviour. Like the WP-C synthetic-cloud generator, it is a test fixture, not the deferred SITL work (masterplan §5, §10).

| ID | Scenario | Expected (verified in §18) |
|---|---|---|
| S-01 | Room with a 1.0 m doorway to an unseen room, single plan | One cluster, entirely beyond the doorway; goal reachable, preferred tier, inside the first room with line of sight through the doorway |
| S-02 | Same layout, 0.4 m and 0.6 m doorways, full reveal loop | 0.4 m: never leaves the first room; `DONE` with ≥ 1 unexplored cluster. 0.6 m: grid-traversable (§7.8); second room explored, `DONE` with 0 unexplored |
| S-03 | Room plus 8.4 m × 1.2 m dead-end corridor | Goes to the corridor end in ≈ 2.4 m steps; `DONE` with 0 unexplored; no free cell left unknown |
| S-04 | Enclosed room, fully visible from start | No goals; `DONE` after `done_confirm_cycles` |
| S-05 | Free room with random 1–4-cell unknown speckles | No clusters |
| S-06 | Start room between a 2 m corridor (west room) and a 5 m corridor (east room); corridors 0.8 m and 1.2 m | Nearer (west) frontier first; `DONE` with 0 unexplored and no free cell left unknown. 0.8 m uses tight-tier goals |
| S-06b | S-06 with 0.8 m corridors and `allow_tight_goals = false` | `DONE` with ≥ 1 unexplored cluster (justifies H-19) |
| S-07 | Map mirrored exactly about the robot | Exact utility tie; lower label (west) chosen; identical across runs |
| S-08 | Unknown only outside the planning box | No goals |
| S-09 | Commitment against the current goal | `U_cur` of the selected goal equals its `U`; no switch within `switch_margin` |

### 13.3 Property tests (hypothesis)
| ID | Property over random grids, poses, blacklists |
|---|---|
| T-P01 | Every emitted goal: `check_goal(B, infl)` passes; FREE; `clr ≥ traverse_clearance_m`; `d_unk ≥ traverse_clearance_m`; if `goal.tight` is false also `clr ≥ goal_clearance_m` and `d_unk ≥ unknown_standoff_m`; reachable by an independent BFS with the same no-corner-cutting rule |
| T-P02 | With no `unk_eff` cells, no goal is ever emitted |
| T-P03 | Identical inputs give identical decisions (goal, utility, cluster, tier) |
| T-P04 | No goal lies within an active blacklist radius |

### 13.4 Node and integration tests
| ID | Test |
|---|---|
| T-N01 | In-process fake publishers (pose, platform info, LIO, fence) and a fake QueryMap server: START ⇒ WAIT_MAP ⇒ goal published on `/go_to_with_avoidance/goal` with the correct frame, z and yaw |
| T-N02 | Remote parameter checks: fake shim, map and fence parameter services; a mismatch refuses START with the right message |
| T-N03 | Fake QueryMap stops answering ⇒ I5 after `map_fail_max` cycles; hold goal and hover request published |
| T-N04 | LIO DEGRADED ⇒ PAUSED; no goal or hover published by the explorer |
| T-N05 | Fence heartbeat stops ⇒ PAUSED(I4) within `fence_heartbeat_timeout_s` + 0.1 s |
| T-N06 | The shim's real alert strings, copied from `go_to_with_avoidance_node.py`, are classified correctly |
| T-N07 | `offboard = false` ⇒ PAUSED(I2); no goals |
| T-N08 | Explorer killed: no further publications on the goal topic |
| T-N09 | Status JSON validates against §4.5; heartbeat at 1 Hz ± 10%; one alert per state or reason change |
| T-N10 | Graph introspection of the running node: its publishers, service servers and service clients are exactly those in §4.2–§4.4 |
| T-I01 | **Smoke** (`tools/replay_tests`): real `map_server_node` fed with `synthetic_scene` clouds plus a doorway variant, the real `planner_shim` node, fake AS2/LIO/fence publishers. Assert a goal is published, the shim accepts it (no REJECTED alert), and the goal lies inside the room. Open loop only; no motion. |

### 13.5 Benchmarks (recorded in the compute ledger)
| ID | Measurement |
|---|---|
| B-01 | QueryMap latency for N = 2,116 / 10,000 / 40,000 in one cycle: warm EDT (median, p95, max), chunked, and post-insertion |
| B-02 | Planning time (§7.4–§7.14) on synthetic grids for N = 2,116 / 10,000 / 40,000 with 1, 8 and 32 clusters |
| B-03 | CPU and RSS of the node during T-I01 over 5 minutes |

## 14. Verification debt to open (masterplan §1.3)

```yaml
  - id: VD-007
    package: wp-f
    claim_untested: "frontier_explorer closed loop: explorer goal -> planner_shim -> EGO -> AS2 -> PX4 offboard; arrival, no-progress and timeout detection against real vehicle motion; hold goal and hover-request consumption"
    original_criterion: "closed-loop exploration run (masterplan §8 cap: BUILT_UNVERIFIED)"
    discharge_path: "gate7 exploration session after gate6 token, executed by the human from an agent-written runbook (tethered first, masterplan §1.1 rule 4), or SITL later; unit, scenario, property and open-loop smoke tests executed and passing"
    status: OPEN
  - id: VD-008
    package: wp-f
    claim_untested: "QueryMap slice latency and explorer planning time on REAL Mid-360 /cloud_registered data at venue-sized boxes (benchmarks B-01/B-02 use synthetic clouds)"
    original_criterion: "PR-1/PR-2 on recorded data"
    discharge_path: "re-run B-01/B-02 against Gate 2/3 bags"
    status: OPEN
  - id: VD-009
    package: wp-f
    claim_untested: "frame consistency threshold (I6): AS2 earth-frame pose vs FAST-LIO odom under EKF2 EV fusion; default 0.5 m untuned"
    original_criterion: "measured divergence distribution during Gate 3 hand-carry"
    discharge_path: "Gate 3 log review; set frame_divergence_max_m from observed p99 + margin"
    status: OPEN
```

## 15. Phase 2 hooks (not implemented)
- `band_half_layers > 0`: sample `2·h + 1` layers. Frontier test becomes 6-neighbour in 3D within the band; clusters 26-connected.
- Candidate goals get `z ∈` layer centres. `check_goal` and C5 are applied per layer.
- **Descent rule:** a goal below the current altitude is admissible only if every cell in its column from the current layer down to the goal layer is `FREE` (observed), because the Mid-360 cannot see directly below and `clr` treats unknown as free.
- Visibility rays become 3D segments with the same endpoint rule.
- C1 changes from "must be 0" to a range. All Phase 1 tests must still pass with `band_half_layers = 0`.

## 16. Open items
| ID | Item | Owner | Due |
|---|---|---|---|
| OI-1 | **Resolved 2026-09-13: deviation declined.** No WP-F implementation until `map_interface` is merged to `main` after the `gate5_as2_behaviors` token; `wp-f-frontier` is then created from `main` | Operator | Done |
| OI-2 | Planning box, map bounds and geofence defaults are mutually inconsistent (C4 fails today) | Operator: size for the venue | Before Gate 7 |
| OI-3 | Shim has no structured goal status; rejection detected by string prefix | Future WP-D change (not this WP) | Before Gate 7 |
| OI-4 | Consumer of `/go_to_with_avoidance/hover_request` unverified (shared with VD-006) | Gate 6 integration | Gate 6 |
| OI-5 | EGO/AS2 handling of goal orientation (yaw hold) | [VERIFY] | Gate 6 |
| OI-6 | EKF2 EV origin offset vs LIO world | [VERIFY] | Gate 3 |
| OI-7 | **Resolved 2026-09-13.** `decision_dp2_explore: FRONTIER` is recorded in `gate_status.yaml` (commit pending), and masterplan amendment A1 (operator-authorized) adds FRONTIER to §0, §2 and §8 | Operator | Done |
| OI-8 | PR-2 budget set from synthetic measurements | VD-008 | Gate 6 |
| OI-9 | Transparent obstacles (glass) undetectable; single-box geofence cannot exclude regions | Operator: venue survey | Before Gate 7 |
| OI-10 | `PlatformInfo.offboard` semantics with `as2_platform_pixhawk` (I2, P2) | [VERIFY] | Gate 5 |
| OI-11 | `wp-d-planner` `drone_bringup/setup.py` does not install `config/planning/` (`ego_v2.yaml`) | Fix on `wp-d-planner` | Before `wp-d` merges to `main` |

## 17. Implementation plan (Step → Verify → Check)
| # | Step | Verify | Check |
|---|---|---|---|
| 1 | After `map_interface` and `planner_shim` reach `main` (OI-1), create `wp-f-frontier` from `main` | `colcon build --parallel-workers 2` (`MAKEFLAGS=-j4`) of `map_interface`, `planner_shim`, `drone_bringup` | Existing WP-C/WP-D tests still pass; `ego_v2.yaml` installed (OI-11 fixed on `wp-d-planner`) |
| 2 | `config.py` + T-C* | pytest | C4 fails on today's defaults (OI-2 reproduced) |
| 3 | `grid.py` + T-G* | pytest | 46×46 at defaults; exact-arithmetic sweep passes |
| 4 | `frontier.py` + T-F* | pytest | S-04, S-05, S-08 frontier-level |
| 5 | `planner.py` + T-S*, T-P* | pytest + hypothesis | B-02 within PR-1 (vectorised §7.12) |
| 6 | `lifecycle.py`, `fsm.py` + T-L*, T-M*, S-* | pytest | Every §9.2 row covered |
| 7 | `explorer_node.py` + T-N*, T-U* | pytest (localhost DDS) | Heartbeat, services, graph contract |
| 8 | Launch, config, smoke T-I01, B-01, B-03 | replay test | Compute ledger row added |
| 9 | VD-007…009, pinned versions, lint | `ament_flake8`, `ament_pep257` | Commit declared BUILT_UNVERIFIED (DP-2 decision recorded) |

Estimated effort: 1–2 weeks. The §7 prototype in `tools/frontier_prototype/` (§18) cuts steps 3–5 roughly in half, because it is a verified reference implementation of the algorithm.

## 18. Verification record

Verification artefacts are saved in `tools/frontier_prototype/` (see its README): `proto.py`, `test_proto.py`, `bench_query.py`, `fact_check.py`, `xref_check.py`, plus provenance-stamped copies of `shim_core.py` (from `wp-d-planner`) and `synthetic_scene.py` (from `wp-c-mapping`). The prototype is a literal implementation of §7 used to test this document's claims; it covers neither §8–§9 nor the ROS node, and it is **not** the product code. All scripts were re-run from that location on 2026-09-12 with the same results.

### 18.1 Pass 1: facts and behaviour
| Check | Method | Result |
|---|---|---|
| Interface facts | 52 claims (node names, topics, types, QoS, defaults, enum values, alert strings, masterplan quotes, unused VD IDs, installed deps) regex-checked against `wp-c`, `wp-d`, `wp-e`, `main` sources and installed AS2 headers | 52/52 after fixing one over-strict regex (F27); F48 confirmed the OI-11 defect |
| QueryMap latency (B-01) | Installed `map_server_node`, `wp-c` synthetic room (26,784 points), localhost DDS | PR-2 numbers in §11 |
| Algorithm behaviour | Literal prototype of §7 with the shim's real `check_goal`; scenarios S-01…S-09, grid sweeps, C4, hypothesis properties (340 examples) | 42/42 after the corrections below |
| Planning time (B-02) | Prototype, lattice maps | PR-1 numbers in §11 |

Defects found in v1 and corrected in v2:
1. **§7.12 visibility:** samples next to an endpoint rounded onto the endpoint cell, so rays to walls were always blocked and phantom frontiers appeared around every room. Fix: endpoint cells never block.
2. **§7.12 goals in narrow corridors:** with a 0.6 m clearance and 0.6 m standoff, no goal fits a 0.8 m corridor, so connected rooms stayed unexplored. Fix: tiered candidates plus `allow_tight_goals` (S-06, S-06b).
3. **§7.12 corridor creep:** tie-breaking by path distance produced 13 goals along an 8.4 m corridor. Fix: tie-break by proximity to the cluster (4 goals).
4. **§7.14 ambiguity:** which cluster's gain defines `U_cur` was undefined. Fix: maximum over clusters where the goal is a candidate, uncapped.
5. **§7.2 false example:** `−4.5/0.2` is exactly `−22.5` in IEEE doubles and drops nothing. Replaced with verified cases (`1.2/0.2`, box edges at 0.1 m).
6. **§7.8 missing discretisation note:** centre-to-centre clearance makes a 0.6 m door grid-traversable at 0.4 m. Documented, with the NO_PROGRESS path.
7. **§11 wrong board condition:** the board runs MAXN_SUPER / schedutil, not "15 W + jetson_clocks". PR-2 budget tightened from 500 ms to 100 ms from measurements.
8. **§12 install gap:** `drone_bringup/setup.py` subdirectory rule and the `wp-d` `config/planning` omission (OI-11).
9. **§6 C3:** added `map_backend == cpu_grid`, since DP-1 is decided.

### 18.2 Pass 2: consistency, compliance and safety
| Check | Method | Result |
|---|---|---|
| Internal consistency | `xref_check.py`: § references, parameter definitions and use, requirement → test traceability, test existence, state-machine contiguity / reachability / absorbing FAULT, interlock rows against SR-4 and SR-10, C/P/H/OI/VD ID integrity, JSON and YAML blocks, numeric relations between defaults, the OI-2 claim | v2: 2 real defects (below) plus 3 checker false positives (IDs embedded in test names), fixed in the checker. v3: first run 53/54, failing because the compliance table below cited masterplan sections without naming the masterplan; after fixing that, **54/54** |
| Fact re-check | `fact_check.py` re-run | 52/52 |
| Reproducibility | Prototype suite re-run | 42/42; B-02 within 3% of pass 1 |
| `map_backend` remote read | Source inspection of `map_server_node.cpp` | Declared unconditionally; the server throws on non-`cpu_grid` (C3 is defence in depth) |
| Masterplan compliance | Rule-by-rule review (table below) | 2 defects (below) |
| Safety review | Every interlock row, every hold-goal publisher, every hazard against its detection and test | 3 defects (below) |

Defects found in v2 and corrected in v3:
1. **Traceability:** S-01, S-05 and T-P02 were not traced to any requirement; T-S07 claimed PR-1 without Appendix B listing it.
2. **SR-10 hole:** `stop` (T20), `pause` (T17) and `on_done` could publish a hold goal while I3 or I4 was raised, competing with the LIO guard or the watchdog. Fix: §8.6 suppresses hold goals while I3/I4 are raised; new T-M09.
3. **Status cap:** masterplan §8 caps WP-F at EVIDENCE_ONLY until the DP-2 decision; v2 claimed BUILT_UNVERIFIED unconditionally. Fixed in the header, §1.3 and §17.
4. **SR-2 untested for the node module:** T-U01 only covers core purity. Added T-U02 (static contract scan) and T-N10 (graph introspection), plus H-20.
5. **Interlocks in PAUSED:** conditions were evaluated only in active states, so T18/T19 had nothing current to decide on. Now evaluated in every state; transitions only from active states; tick evaluation order defined.
6. **Minor:** `SELECTING` described as transient despite T6; alerts on transitions untested (T-M01, T-N09 extended); mission timeout across `PAUSED` undefined (§7.18); VD-007 lacked the human-runbook clause (masterplan §1.1 rule 4); Gates 1–4 global rules not restated (§12).

Masterplan compliance:
| Masterplan rule | Where satisfied in this spec |
|---|---|
| masterplan §1.1 tokens; agent never edits `gate_status.yaml` | §1.3, OI-7 |
| masterplan §1.1 rule 3: one branch per WP; `main` only on token | §1.3 |
| masterplan §1.1 rule 4: vehicle steps are human-executed runbooks | §14 VD-007 |
| masterplan §1.2 ENU planning frame; one estimator of record; CycloneDDS; compute ceiling | §2.1, §9.5, §4, §11 |
| masterplan §1.3 VD opened for closed-loop logic that cannot be executed | §14 |
| masterplan §1.4 status vocabulary; masterplan §8 cap | header, §1.3, §17 |
| masterplan §6 one command path through `go_to_with_avoidance` | §0, §3, SR-2, T-U02, T-N10 |
| masterplan §8 never talks to PX4; execution through the AS2 behaviour path; geofence native to the planner AND enforced by WP-E; preconditions | §1.2, §3, §7.12 step 4 and C4 (native) plus `geofence_watchdog` (independent), §1.3 |
| masterplan §10 no sim; no vehicle testing in place of sim; no multi-drone; no third-party edits; no self-certification | §1.2, §13.2, §17, §18 (evidence only, no gate claims) |
| Gates 1–4 handover global rules: path header, colcon limits, no RViz on the vehicle | §12, §17, §4.2 |

## Appendix A. Scenario fixtures

All fixtures are band maps at 0.2 m (one cell = 0.2 m), index `[i, j]` with `i` = x (east), `j` = y. Cells not listed are `OCC`. All walls are at least 2 cells thick, so raycast rounding cannot leak diagonally. `clr` is computed from known `OCC` cells with a 2D Euclidean distance transform, unknown treated as free (matching the server), clamped at 4.0 m. The planning box is the whole grid.

| Fixture | Grid | Free space | Robot |
|---|---|---|---|
| `room_with_door(w)` | 52 × 18 | Room A i 3–22, j 3–14 (4.0 × 2.4 m); door through wall i 23–24 at j `9 − w//2 … 9 − w//2 + w − 1`; room B i 25–48, j 3–14 | (13, 9) |
| `corridor` | 70 × 20 | Room i 3–22, j 3–16; corridor i 23–64, j 7–12 (8.4 × 1.2 m), dead end | (10, 10) |
| `enclosed` | 24 × 18 | Room i 3–20, j 3–14 | (12, 9) |
| `speckle` | 40 × 40 | All free except a 2-cell border wall; 25 random 1–4-cell unknown patches (seed 3); fully known otherwise | (20, 20) |
| `two_rooms_corridors(w)` | 90 × 22 | Room W i 3–11, j 5–16; west corridor i 12–21; start room i 22–35, j 4–17; east corridor i 36–60; room E i 61–85, j 3–18; corridors at j `11 − w//2 … 11 − w//2 + w − 1` | (25, 11) |
| `symmetric` | 61 × 18 | Room i 20–40, j 3–14; corridors i 3–19 and i 41–57, j 7–10; observed map mirrored exactly about i = 30 after reveal | (30, 9) |

## Appendix B. Requirement traceability
| Req | Statement | Tests |
|---|---|---|
| FR-1 | Sample the band via chunked QueryMap every cycle | T-G01, T-G02, T-G03, T-G04, T-N01 |
| FR-2 | Frontier = FREE ∧ 4-neighbour of effective unknown | T-F01, T-F03, T-F04, T-P02, S-01, S-05 |
| FR-3 | 8-connected clusters with size and count limits | T-F01, T-F04, T-S07 |
| FR-4 | Dijkstra reachability without corner cutting; robot snapping | T-S01, T-S05 |
| FR-5 | Tiered candidates, visibility and gain per §7.12 | T-S02, T-S03, T-S08, T-P01, S-01, S-06, S-06b |
| FR-6 | Utility and commitment per §7.13–§7.14 | T-S04, T-S06, T-S09, S-07, S-09 |
| FR-7 | Goals only to `/go_to_with_avoidance/goal`, pre-validated | T-N01, T-P01, T-I01 |
| FR-8 | Goal lifecycle per §8.2 | T-L01, T-L02, T-L03, T-L04, T-L05, T-L07, T-N06 |
| FR-9 | Blacklist per §8.5 | T-L06, T-P04 |
| FR-10 | Termination per §7.18 | T-F05, T-M08, T-P02, S-02, S-03, S-04 |
| FR-11 | Operator services per §4.3 | T-M01, T-M05, T-M07, T-N02 |
| FR-12 | State heartbeat, status JSON, alerts | T-M01, T-N09 |
| FR-13 | Determinism | T-S06, T-P03, S-07 |
| SR-1 | Config consistency C1–C7 | T-C01, T-C02, T-C03, T-C04, T-C05, T-C06, T-N02 |
| SR-2 | No `/fmu/*`, arming, takeoff, landing, modes, velocities; publishers exactly per §4.2 | T-U02, T-N10 |
| SR-3 | Every goal passes `check_goal` and its tier's clearance and standoff | T-S02, T-P01 |
| SR-4 | No goal unless FLYING | T-M04, T-N07 |
| SR-5 | Interlocks per §9.3, latched PAUSED | T-M02, T-M03, T-M05, T-M06 |
| SR-6 | No command streaming; ≤ 1 re-publish per goal | T-L05, T-N08 |
| SR-7 | No automatic start; START preconditions | T-M07 |
| SR-8 | Band restriction; unknown outside the grid ignored | T-F02, S-04, S-08 |
| SR-9 | Frame consistency monitor | T-F06 |
| SR-10 | No goals or hover while LIO or watchdog own the response, including on `stop`/`pause`/`on_done` | T-M02, T-M09, T-N04, T-N05 |
| IR-1 | Interfaces exactly per §4 | T-N01, T-N02, T-N03, T-N04, T-N05, T-N06, T-N07, T-N08, T-N09, T-N10 |
| IR-2 | Standard message types only | T-U02 |
| PR-1 | Planning time | B-02, T-S07 |
| PR-2 | QueryMap latency | B-01 |
| PR-3 | CPU | B-03 |
| PR-4 | RSS | B-03 |
| VR-1 | Core purity | T-U01 |
