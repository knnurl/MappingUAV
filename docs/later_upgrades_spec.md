<!-- filepath: docs/later_upgrades_spec.md -->
# Later upgrades: implementation spec

| | |
|---|---|
| Status | DRAFT v1, 2026-11-16. For operator review; nothing here is implemented. |
| Scope | Three upgrades to do **after** Gate 7 passes with the current stack: U-A exploration at varying altitude, U-B reactive planning with EGO-Planner, U-C a globally consistent final map. |
| Order | U-C first (it only touches what happens after landing), then U-A, then U-B. Reasons in §4. |
| Governing docs | Masterplan §1 (tokens, branches, debt), §5, §6, §8; `docs/wp-f_frontier_explorer_spec.md` §15 (Phase 2 hooks). |
| Status cap | Each upgrade is BUILT_UNVERIFIED until flown. Each needs an operator decision to start, because each changes something a gate has already passed with. |

Conventions: frames are odom/ENU, metres. "Explorer" is `frontier_explorer`, "planner" is
`go_to_with_avoidance` (design B), "map" is `map_interface`. [VERIFY] marks a claim to check against
source before building on it. For each step the key decisions are stated.

What exists today and is reused: the map is already 3D (octomap + 3D distance field); only the planner and
the explorer look at one horizontal slice. The explorer's core is pure Python with scenario tests; the
closed-loop rig (`sim_bringup`) runs the whole stack. Every upgrade below must keep the rig's mission and
its four failure scenarios passing.

---

## U-A. Exploration at varying altitude

### A.1 Why, and what it does not do

Today the explorer finds frontiers in one layer (`z_fly`, 1.3 m). Space above or below that layer is
mapped only as far as the Mid-360 happens to see it (−7° to +52° elevation): the floor near the vehicle
and anything under a table or above a cabinet stay unknown. U-A lets the explorer choose goals in a band
of layers so those regions get seen.

It does not make the vehicle fly under furniture. Flyable height stays bounded by the planning box and by
clearance. It does not change who avoids obstacles.

### A.2 Requirements

| ID | Requirement |
|---|---|
| A-FR1 | Frontiers are extracted in `2·h + 1` layers around `z_fly` (`band_half_layers = h`, today forced to 0). |
| A-FR2 | A frontier cell is FREE with a 6-neighbour (4 in-plane, up, down) in effective unknown. Clusters are 26-connected across layers. |
| A-FR3 | Candidate goals may lie in any band layer whose centre satisfies C5 (`z_min + infl ≤ z ≤ z_max − infl`). |
| A-FR4 | Gain counts frontier cells visible along 3D rays, limited to the sensor's elevation range as seen from the candidate: a cell below −7° or above +52° from the viewpoint is not visible. |
| A-SR1 | **Descent rule.** A goal below the current altitude is admissible only if every cell of the column from the current layer down to the goal layer is observed FREE with clearance. The LiDAR does not see straight down and the distance field treats unknown as free. |
| A-SR2 | Ascent uses the same rule upward. |
| A-SR3 | With `band_half_layers = 0` every decision is bit-identical to today's (all current tests pass unchanged). |
| A-PR1 | Planning time ≤ 1.0 s per cycle at 5 layers × 10,000 cells. |

### A.3 Design

Key decision: **stack of 2D layers, not a general 3D planner.** Reachability stays a 2D Dijkstra per layer
plus vertical edges between vertically adjacent traversable cells. That keeps the verified 2D code and
makes A-SR1 a property of the graph: a vertical edge exists only between two observed-free cells.

| Module | Change |
|---|---|
| `grid.py` | `Grid` gains `k_z_min..k_z_max`; `query_points()` returns layers in order; arrays become `(nz, nx, ny)`. Chunking unchanged. One cycle queries `nz` times the points: at 46 × 46 × 5 = 10,580 points, three chunks. |
| `frontier.py` | Unknown filter per 3D connected region (6-connectivity; `min_unknown_region_cells` becomes a volume count, default 27). Frontier mask by 6-neighbour dilation. `scipy.ndimage` handles 3D with the same calls. |
| `planner.py` | `dijkstra` over the layered graph: in-plane moves as today, vertical moves cost `res`, allowed only if both cells are traversable. Visibility rays in 3D with the same endpoint rule, plus the elevation test of A-FR4. Utility unchanged. |
| `fsm.py`, `lifecycle.py` | Goal carries its own z. `reach_tolerance_z_m` already exists. |
| `config.py` | C1 accepts `band_half_layers` in `[0, 3]`; C5 checked for every band layer; C6 counts all layers. |
| planner node | Already handles a goal at another altitude: it checks the column at the start cell and climbs or descends in place first. Change: allow the vertical move at the **goal** end as well when the column there is observed free and the start column is not, and choose the cheaper. |

Key decision: **vertical moves happen in place** (pure climb or descent), never diagonally. A diagonal
through layers would need 3D clearance along a slanted segment and a 3D path check; in-place moves reuse the
column check that already exists and is tested.

Key decision: **sensor elevation in the gain** (A-FR4). Without it the explorer would send the vehicle to
hover directly over an unknown floor patch it cannot see from there. With it, the best viewpoint for a floor
patch is a few metres away horizontally, which is physically right.

### A.4 Parameters (new or changed)

| Name | Default | Purpose |
|---|---|---|
| `band_half_layers` | 0 (off) | Layers above and below `z_fly`. 2 gives 0.9 to 1.7 m at 0.2 m resolution. |
| `min_unknown_region_cells` | 9 (27 when the band is on) | Speckle threshold, area or volume. |
| `sensor_elev_min_deg`, `sensor_elev_max_deg` | −7, 52 | Mid-360 field of view for A-FR4. |
| `vertical_cost_factor` | 2.0 | Cost multiplier for vertical moves: altitude changes are slower (0.5 m/s limit) and cost more battery. |

### A.5 Tests

- Every existing test with `band_half_layers = 0`: unchanged results (A-SR3).
- Unit: 6-neighbour frontier; 26-connected clusters; vertical edge only through observed-free cells;
  elevation test (cell at −30° not visible, at −5° visible); C5 per layer.
- Scenarios (3D fixtures, same open-loop reveal with a 3D raycast limited to the sensor's elevation range):
  a table (occupied slab at 0.7 to 0.8 m) with unknown beneath: the explorer never descends under it and
  reports the region as unexplored; a low wall (1.0 m) hiding floor behind it: a viewpoint above 1.2 m sees
  over it and clears it; a room with a raised unknown shelf: seen from a lower layer.
- Property: no goal whose column to the current altitude contains an unknown or occupied cell.
- Rig: a world with a low partition; mission ends DONE with the area behind the partition known.

### A.6 Verification debt and gate

New entry: closed loop with altitude changes on the vehicle (rangefinder and terrain-hold interaction:
`MPC_ALT_MODE`, `EKF2_RNG_CTRL` behaviour when flying over furniture is unverified and matters here).
Flight test belongs in a repeat of the Gate 7 runbook Part C with `band_half_layers = 1` first.

### A.7 Effort and risk

About one week. Main risk: height estimation over uneven ground. Flying over a table makes the downward
rangefinder read 0.5 m less; with terrain hold that is a climb. Settle the height source question (Gate 4
checklist, "HEIGHT PATH DECIDED") before this upgrade flies.

---

## U-B. Reactive planning with EGO-Planner

### B.1 Why

Design B is a static-scene planner: it re-checks its path once a second against a map updated twice a
second and flies only through cells already seen free. It cannot dodge anything that appears, it stops
rather than flowing round an obstacle, and its paths are polylines with stops at corners. EGO-Planner keeps
its own local map from the live cloud and re-plans a smooth trajectory at about 10 Hz.

What it costs: a second map in memory, a C++ planner tuned for depth cameras fed with a sparse LiDAR
cloud, a trajectory interface into Aerostack2 that the masterplan currently forbids ("do not enable
Aerostack2 trajectory paths yet", §7), and a larger failure surface. Do it only when a measured need
exists: Gate 7 flights limited by stop-and-go motion, or a venue with moving obstacles.

### B.2 Decision gate before any code

One bounded session of evidence, written to `docs/dp3_planner_evidence.md`:

1. Build the pinned `ego-planner-swarm` (23a8d5a1, `ros2_version`) in a scratch workspace. Record from
   **source and a dry run**: node names, every topic with its type, the goal input, the odometry input,
   the map input (cloud or depth, expected frame and rate), and the command output. [VERIFY: the trajectory
   server publishes a position command message with position, velocity, acceleration and yaw on a
   per-drone topic; the planner takes a point cloud in the world frame.]
2. Feed it replayed `/cloud_registered` and `/Odometry` from a Gate 6 bag: does its local map fill sensibly
   from a 10 Hz non-repetitive cloud? CPU and memory on the Orin.
3. In Aerostack2 1.1.3, which behaviour or motion reference accepts a position + velocity + acceleration
   reference, and what the platform sends to PX4 for it. [VERIFY: `as2_platform_pixhawk` supports POSITION
   and SPEED control modes only (its `control_modes.yaml`), so a trajectory reference is tracked by the
   Aerostack2 controller and reaches PX4 as velocity setpoints.]

Stop if EGO's map does not fill from the LiDAR cloud, or if planner + map exceed the 2.0-core ceiling
together with the rest. Operator decides (a new decision point, DP-3: GRID_PATH or EGO).

### B.3 Design (if DP-3 = EGO)

Key decision: **keep the node name, topics and safety logic of `go_to_with_avoidance`; replace only what is
between an accepted goal and the Aerostack2 action.** The explorer, the hover consumers, the gating rules
(`stop_reason`), the status topic and the contract test stay.

```
goal ─► go_to_with_avoidance ─(validated goal)─► EGO planner ─(trajectory samples)─► go_to_with_avoidance
              │  gate: LIO, platform, pose, box                                          │ check, convert
              │  coarse check: goal cell observed free (map_interface)                    ▼
              └────────────────────────────────────────────── Aerostack2 follow-reference / trajectory
```

| Piece | Design |
|---|---|
| Coarse layer | The grid A* stays, as a **global guide**: EGO is local (5 m horizon) and gets stuck in dead ends. The A* path is cut into intermediate goals about 3 m apart; EGO flies between them. Key decision: this keeps "never plan through unseen space" true at the global level even though EGO's local map treats unknown as free. |
| Unknown space | EGO's grid map inflates obstacles but does not avoid unknown cells. Mitigation: intermediate goals only on observed-free cells; planning horizon ≤ `view_range_m`; and the 1 Hz check of the next 3 m against `map_interface` stays, cancelling on a blocked or unknown cell. |
| EGO inputs | `/cloud_registered` remapped to its cloud topic, `/Odometry` to its odometry topic (both already world-frame). No second estimator: EGO uses LIO for its map only, never for control. |
| Trajectory to Aerostack2 | Samples at 30 Hz converted to an Aerostack2 trajectory reference; `shim_core.check_trajectory` (exists, tested, unused today) validates each window: inside the box, speed and acceleration limits, monotonic time. A failed window cancels and holds. |
| Stop | Same as today: cancel the Aerostack2 behaviour; the rig shows that holds. Plus an explicit stop command to EGO so it does not keep publishing. |
| Limits | `max_vel` 1.0, `max_acc` 2.0, inflation 0.4, horizon 5.0 (`config/planning/ego_v2.yaml`, already there). |
| Fallback | Parameter `planner_mode: grid_path | ego`. Grid path stays the default and the fallback: if EGO produces nothing for 2 s, the node cancels and reports FAILED; it does not switch modes in flight. |

Masterplan impact: §6 already names EGO; §7's "no trajectory paths yet" must be lifted by the operator for
this upgrade. The single-command-path rule holds: EGO never talks to PX4.

### B.4 Tests

- Unit: trajectory sample conversion; `check_trajectory` windows; intermediate goal spacing; fallback timer.
- Node (fake EGO publishing canned trajectories): accepted, rejected for leaving the box, rejected for
  overspeed, EGO silent → FAILED after 2 s.
- Rig with the real EGO: goal round the pillar world; goal through the door; an obstacle inserted into the
  fake world mid-flight (new rig hook) is avoided or the vehicle stops; all four existing scenarios pass.
- Compute ledger rows for EGO planner and its map.

### B.5 Debt, gate, effort, risk

Debt: closed loop on the vehicle (a new Gate 6 run: repeat `docs/gate6_planner_runbook.txt` Part D with
`planner_mode: ego`, plus a slowly carried obstacle). Effort: 2 to 3 weeks including the evidence session.
Risks: EGO's map quality from a sparse rosette cloud (the reason for step B.2.2); tracking error of the
Aerostack2 controller on a continuously moving reference with untuned gains; EGO treating unknown as free.

---

## U-C. A globally consistent final map

### C.1 Why

The mission's product is the 3D map. Today it is the cloud as FAST-LIO registered it: locally sharp,
globally drifting (9.5 to 13 cm over a 103 m loop on the 2026-10-23 recording). The LIO fork has loop
closure (`pgo`) and offline refinement (`hba`). U-C uses them **without letting any correction reach the
flight controller**.

### C.2 The rule that shapes everything

**Nothing that enters EKF2 may jump** (LIO roadmap D3, fork roadmap B2). The odometry, the occupancy map
the planner and explorer use, the geofence and the EV guard all stay in the uncorrected odometry frame for
the whole flight. Corrections exist only as a `map → odom` transform and in files written for after the
flight. The fork enforces the first half by construction and now tests it (`tools/eval/test_no_jump_rule.py`).

Consequence, accepted: during a flight the planning map drifts with the odometry. In a venue of tens of
metres that is centimetres to a decimetre, inside the 0.4 m inflation. A revisited wall may appear doubled
in the occupancy map by that much; octomap's probabilistic update absorbs it.

### C.3 Design, in three stages (each useful alone)

**C-1: record what refinement needs (in flight, cheap).** Refinement needs per-scan poses and per-scan
clouds, not just the merged map. `map_saver_node` gains a second output: every `keyframe_distance_m`
(0.5 m) or `keyframe_angle_deg` (15°), write the current scan (body frame, voxel-thinned to 0.1 m) and its
odometry pose as `keyframes/<n>.pcd` + a line in `poses.txt`. Bounded by `max_keyframes`. Key decision:
body-frame scans, because a corrected pose can then re-place them; the world-frame chunk output stays as
the quick-look product.
**Built 2026-11-17** in `map_saver_node` (`keyframes/kf_<n>.pcd` + `poses.txt`). Measured on the hallway
recording through the fork (21.6 m of path): 95 keyframes, about 5,000 points and 60 kB each, 5.7 MB in
total; scans re-placed with their poses line up (median nearest-neighbour distance 0.15 to 0.24 m between
scans 0.6 to 1.6 m apart, against 0.8 to 1.7 m without the poses). `map_saver_node` used 11 % of a core fed
with the fork's full-resolution world cloud (1.3 % with upstream's downsampled cloud, without keyframes). Needs FAST-LIO's body-frame cloud: upstream has
`scan_bodyframe_pub_en`, the fork has `publish_body_cloud`; each costs a cloud copy per scan (measure).

**C-2: offline refinement (after landing, on the Jetson or the ground station).** A script
`tools/map_refine/refine.py`: loads keyframes and poses, runs the fork's `pgo` (loop detection + pose
graph) and then `hba` (bundle adjustment of planes) through their existing services (`SavePoses`,
`RefineMap`) [VERIFY the service contracts and whether they run headless from files], writes
`poses_refined.txt`, re-places every keyframe, voxel-filters to the product resolution and writes
`map_refined.pcd`. Then the existing `map_export` post-processing (downsample, optional mesh).
Quality numbers written beside the map (fork roadmap C1): loop gap before and after; wall thickness
(RMS distance to fitted planes on three large walls); floor thickness.
Key decision: **offline and optional.** A failed refinement leaves the unrefined map intact; the script
never overwrites inputs.

**C-3: loop closure visible in flight (map frame only).** Run `pgo` during the flight so the ground station
sees a corrected map and trajectory in the `map` frame, and so the saved keyframe poses can be stored
already corrected. `map → odom` stops being the identity static TF: remove
`tf_map_to_odom` from the launch files when `pgo` runs, since two publishers of the same transform fight.
Nothing on the vehicle may **consume** `map`: add a contract test over the autonomy packages (no
`lookupTransform` to `map`; explorer, planner, watchdog, bridge and Aerostack2 stay on `odom`/`earth`).
Compute: `pgo` with GTSAM on the Orin alongside the stack must be measured; if it does not fit, C-3 is
dropped and C-2 alone delivers the product.

### C.4 Prerequisites

- The fork is swapped in (LIO roadmap: B1 done 2026-11-16 on a branch, B2 guarded by test, then the swap
  and a re-run of the Gate 2 and Gate 3 checks). C-1 can be built against upstream FAST-LIO; C-2 and C-3
  need the fork's `pgo` and `hba` installed, which does not require the swap for C-2 (offline).
- A loop recording with taped ground truth distances, for the quality numbers.

### C.5 Tests

- C-1: unit tests for keyframe selection (distance, angle, bound) and atomic writes (same pattern as the
  chunk writer); replay on the Gate 2 bag: keyframe count and size as estimated.
- C-2: on the `loop5` recording: loop gap after refinement below the Gate 2 figure (5 cm); wall thickness
  not worse than before; script is idempotent and leaves inputs untouched.
- C-3: the no-jump test on a replay with `pgo` running (largest odometry step unchanged, `map → odom`
  moves at loop closures); the contract test that no autonomy node reads `map`.

### C.6 Debt and effort

Debt: map accuracy against taped distances in the flown venue (Gate 7 runbook Part E already measures
three distances; add the refined map to that comparison). Effort: C-1 two days, C-2 one week, C-3 one week
plus compute evidence.

---

## 4. Order and why

1. **U-C stages C-1 and C-2** first: they improve the deliverable, touch nothing that flies except a small
   recorder, and need no gate to be re-flown.
2. **U-A** next: it changes goal selection only; the planner and safety chain are untouched; the rig covers
   it; one repeat of Gate 7 Part C.
3. **U-B** last and only on evidence: it replaces the motion path, needs a masterplan rule lifted, and
   re-opens Gate 6.
4. **U-C stage C-3** whenever compute evidence allows; it is independent of U-A and U-B.

## 5. Operator decisions this spec needs

| ID | Decision |
|---|---|
| L1 | Start C-1/C-2 after Gate 7, or earlier (C-1 could record during Gate 6 and 7 flights and give refinement data for free)? Recommendation: build C-1 before Gate 6. |
| L2 | U-A: accept in-place vertical moves only, and settle the height source over furniture first. |
| L3 | U-B: run the evidence session (B.2) at all? Recommendation: only if Gate 7 flights show the need. |
| L4 | Lift masterplan §7 "no Aerostack2 trajectory paths" for U-B (masterplan is edit-locked: this would be an operator amendment). |
