<!-- filepath: docs/compute_ledger_wp-b.md -->
# Compute Ledger — WP-B rows (fold into docs/compute_ledger.md at merge)

| Node | CPU (synthetic) | RAM (synthetic) | CPU (vehicle) | RAM (vehicle) | Notes |
|---|---|---|---|---|---|
| AS2 stack (platform + estimator + controller + 4 behaviors, 9 procs) | 18.6% of one core total | 187 MB RSS total | _gate5_ | _gate5_ | measured 2026-07-05, idle graph (no FC, no /fmu traffic); remeasure with live FC at gate5 |

Method: summed /proc utime+stime over a 10 s window after 20 s settle,
max clocks. Counts against the 2.0-core / 2.5 GB Phases-3+ budget together
with map_interface (5.7% / 23 MB) — combined static footprint ~24% of one
core and ~210 MB.
