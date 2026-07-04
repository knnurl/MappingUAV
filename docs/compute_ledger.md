<!-- filepath: docs/compute_ledger.md -->
# Compute Ledger — Jetson Orin Nano 8GB (shared CPU/GPU memory, 6 cores)

Budget [FIXED, masterplan §1.2]: map server + planner + behaviors combined
<= 2.0 cores and 2.5 GB RAM, measured at Gate 6.

Agent fills "synthetic" columns from its own measurements; the HUMAN fills
"vehicle" columns from gate transcripts.

| Node | CPU (synthetic) | RAM (synthetic) | CPU (vehicle) | RAM (vehicle) | Notes |
|---|---|---|---|---|---|
| MicroXRCEAgent | — | — | _gate1_ | _gate1_ | serial 921600 |
| livox_ros_driver2 | — | — | _gate2_ | _gate2_ | CustomMsg mode |
| fastlio_mapping | — | — | _gate2_ | _gate2_ | expect 1–1.5 GB envelope |
| px4_odom_bridge | — | — | _gate3_ | _gate3_ | negligible expected |
| geofence_watchdog | <0.1% of one core | 25 MB RSS | _gate5_ | _gate5_ | measured 2026-07-04, idle graph (no input traffic); remeasure under 10 Hz load at gate5 |
| map_interface (cpu_grid) | _wp-c_ | _wp-c_ | _gate6_ | _gate6_ | counts against 2.0/2.5 budget |
| as2 platform+behaviors | _wp-b_ | _wp-b_ | _gate5_ | _gate5_ | counts against budget |
| ego planner behavior | _wp-d_ | _wp-d_ | _gate6_ | _gate6_ | counts against budget |

Measurement method (keep consistent): `pidstat -h -r -u -p <pid> 30 4`
averaged, steady state, max clocks (drone-clocks.service active).
