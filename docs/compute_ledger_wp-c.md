<!-- filepath: docs/compute_ledger_wp-c.md -->
# Compute Ledger — WP-C rows (fold into docs/compute_ledger.md at merge)

| Node | CPU (synthetic) | RAM (synthetic) | CPU (vehicle) | RAM (vehicle) | Notes |
|---|---|---|---|---|---|
| map_interface (cpu_grid) | 5.7% of one core | 23 MB RSS | _gate6_ | _gate6_ | measured 2026-07-05, synthetic 10 Hz clouds (~15k pts), 2 Hz insertion, EDT every 2 s; counts against 2.0/2.5 budget |
| map_export recorder | not measured | not measured | n/a | n/a | mapping missions only, never during flight autonomy |

Method: /proc utime+stime over a 15 s steady-state window under load,
max clocks.
