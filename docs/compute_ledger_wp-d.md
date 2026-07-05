<!-- filepath: docs/compute_ledger_wp-d.md -->
# Compute Ledger — WP-D rows (fold into docs/compute_ledger.md at merge)

| Node | CPU (synthetic) | RAM (synthetic) | CPU (vehicle) | RAM (vehicle) | Notes |
|---|---|---|---|---|---|
| lio_health_guard | not yet measured under load | ~25 MB expected (same class as watchdog) | _gate6_ | _gate6_ | idle smoke only |
| go_to_with_avoidance shim | not yet measured under load | ~25 MB expected | _gate6_ | _gate6_ | idle smoke only |
| EGO planner chain | not measured (smoke build only) | not measured | _gate6_ | _gate6_ | counts against 2.0/2.5 budget; measure at integration |
