<!-- filepath: docs/compute_ledger_wp-c.md -->
# Compute Ledger — WP-C rows (fold into docs/compute_ledger.md at merge)

| Node | CPU (synthetic) | RAM (synthetic) | CPU (vehicle) | RAM (vehicle) | Notes |
|---|---|---|---|---|---|
| map_interface (cpu_grid) | 5.7% of one core | 23 MB RSS | _gate6_ | _gate6_ | measured 2026-07-05, synthetic 10 Hz clouds (~15k pts), 2 Hz insertion, EDT every 2 s; counts against 2.0/2.5 budget |
| map_export recorder | not measured | not measured | n/a | n/a | mapping missions only, never during flight autonomy |

Method: /proc utime+stime over a 15 s steady-state window under load,
max clocks.

## Real-data replay, 2026-11-16 (map fixes and map saver)

`test_data/2026-09-14_gate2_walk_loop`, recorded `/Odometry` + `/cloud_registered`, 95 s from offset
60 s, real time, isolated domain, one stack at a time. CPU from `top` over 60 s, share of one core.

| Node | CPU | RSS | Notes |
|---|---|---|---|
| map_server_node before the ray fix (a373313) | 4.4% | 20.9 MB | |
| map_server_node with the ray fix + time-matched origin | 7.8% | 20.9 MB | free cells in the z 0.5 / 1.3 m slices: 1744 / 1366 against 1701 / 1309 (small room, mostly inside the box) |
| map_saver_node (0.05 m voxels, 20 s checkpoints) | 1.3% | 18.1 MB | 54,880 voxels, 5 chunks, 668 KB on disk |

Hand-carried bench bag, props off: not a flight measurement (VD-013 stays open for the power-cut test
and the vehicle columns).
