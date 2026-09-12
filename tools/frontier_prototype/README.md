<!-- filepath: tools/frontier_prototype/README.md -->
# `frontier_explorer` reference prototype

Verification evidence for [`docs/wp-f_frontier_explorer_spec.md`](../../docs/wp-f_frontier_explorer_spec.md) (§18), saved 2026-09-12.

**This is not product code and not a ROS package.** There is no `package.xml`, colcon ignores this directory, and nothing here runs on the vehicle. Status: EVIDENCE_ONLY.

The product package (`src/frontier_explorer`, spec §12) is still to be written on `wp-f-frontier`. Use `proto.py` as the reference for **spec §7, the planning algorithm**: it implements the spec text literally, and its tests were run against it. It does **not** implement §8–§9 (goal lifecycle, state machine, interlocks) or the ROS node; those exist only as spec requirements with planned tests.

## Contents
| File | What it is |
|---|---|
| `proto.py` | Literal implementation of spec §7: grid geometry, unknown filter, frontiers, clusters, traversability, snapping, Dijkstra, tiered candidates, visibility and gain, utility, commitment. Also an open-loop reveal script (`World`, `explore`) for scenario tests. It is not a simulator. |
| `test_proto.py` | 42 tests: grid index sweeps against exact arithmetic, the C4 config claim, frontier and planner unit checks, scenarios S-01…S-09 and S-06b, hypothesis properties P01–P04 (checked against independent brute-force references), and the B-02 planning-time benchmark |
| `shim_core.py` | Verbatim copy of `planner_shim`'s `EnuBox` / `check_goal` from `wp-d-planner` (commit in the file header). The prototype needs the real goal check; product code imports `planner_shim` instead. |
| `synthetic_scene.py` | Verbatim copy of WP-C's synthetic room generator from `wp-c-mapping` (commit in the file header), used by `bench_query.py` |
| `bench_query.py` | B-01: times `/map_interface/query` for 2,116 / 10,000 / 40,000-point slices against the built map server |
| `fact_check.py` | Checks 52 spec claims (topics, types, QoS, defaults, enum values, alert strings, masterplan quotes) against the `wp-c`, `wp-d`, `wp-e` and `main` branches and the installed Aerostack2 headers |
| `xref_check.py` | Checks the spec's internal consistency (54 checks). Re-run after every spec edit. |

## Running
```bash
cd ~/colcon_ws/tools/frontier_prototype

# Algorithm tests and B-02 timings (no ROS needed: numpy, scipy, hypothesis)
python3 -m pytest test_proto.py -q -s -p no:cacheprovider

# Spec internal consistency
python3 xref_check.py

# Spec facts against branch sources (needs the wp-* branches locally)
python3 fact_check.py

# B-01 map-query latency (needs map_interface built from wp-c-mapping into ~/colcon_ws/install)
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 python3 bench_query.py
```

## Results when saved (Jetson Orin Nano, MAXN_SUPER, schedutil)
| Script | Result |
|---|---|
| `test_proto.py` | 42 passed |
| `xref_check.py` | 54/54 |
| `fact_check.py` | 52/52 |
| `bench_query.py` | Warm median 5.1 ms (2,116 points), 19.6 ms (10,000), 70.8 ms (40,000); 22.3 ms right after an insertion |
| B-02 (in `test_proto.py`) | 205 ms at 10,000 cells with 32 clusters; 645 ms at 40,000 |

## Caveats
- **Local DDS only.** This environment can't use the WiFi interface named in `cyclonedds.xml`, so the benchmark (and any rclpy test) needs `ROS_LOCALHOST_ONLY=1` with `CYCLONEDDS_URI` unset.
- **Synthetic data.** All timings come from synthetic clouds; real-data numbers are verification debt VD-008.
- **Untracked inputs.** `fact_check.py` checks F42–F47 read the masterplan from the untracked `docs/ProgressReport/`; in a clone without it they report `NOSRC`.
- **Vendored copies can go stale.** `shim_core.py` and `synthetic_scene.py` are snapshots. If `planner_shim` or the WP-C scene changes on its branch, re-copy them; `fact_check.py` does not compare the copies.
