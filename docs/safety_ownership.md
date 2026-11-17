<!-- filepath: docs/safety_ownership.md -->
# Who decides what when LIO goes wrong

Three nodes watch the LiDAR-inertial odometry. Each has one job and one output; none repeats another's
action. Enforced by `tools/contract_tests/test_ownership.py`.

| Node | Looks at | Decides | Output | Reaction it causes |
|---|---|---|---|---|
| `px4_odom_bridge` (EV guard) | each odometry sample; with the fork, the health message of the same stamp | whether THIS sample may reach EKF2 | forwards or withholds it; latches BLOCKED on a jump, a gap over 1 s, garbage values; reports `/px4_odom_bridge/state` | none by itself |
| `lio_health_guard` | odometry rate and jumps; with the fork, per-scan health (degeneracy, match ratio, residual, validity) | whether LIO is DEGRADED | `/lio_health/state`, hover request | planner cancels the behaviour, explorer pauses: the vehicle HOLDS |
| `geofence_watchdog` | PX4's own estimate, odometry liveness, the bridge state | whether to LAND | NAV_LAND to PX4, only while armed, until PX4 confirms | the vehicle LANDS |

Escalation, by design:
1. LIO quality drops but odometry still flows → guard says DEGRADED → hold.
2. Samples stop being forwarded (invalid scans, silence) for over 1 s → bridge reports STALE → watchdog lands.
3. A jump or a restart of LIO → bridge latches BLOCKED → watchdog lands. Only a restart of the bridge clears it.

Rules:
- Only `px4_odom_bridge` publishes external vision. Only `geofence_watchdog` publishes vehicle commands.
  Nothing else in this workspace talks to PX4 except the Aerostack2 platform.
- The planner is the single consumer of the three hover-request topics.
- The explorer never commands; it stops publishing goals when any of the three reports trouble.
- No flight node reads the `map` frame: loop-closure corrections must never reach control.

EV guard changes of 2026-11-17 (spec `docs/px4_odom_bridge_ev_guard_spec.md` §9):
- §9.2 done: the step limit is a budget that grows with the gap (0.30 m at 10 Hz, plus 1.5 m/s for the
  time beyond the nominal period). A rate dip during legitimate motion no longer latches the guard.
- §9.1 in practice: the radius is an implausibility bound that `scripts/venue_boxes.py` sets well beyond the
  venue (at least 20 m); containment belongs to the geofence watchdog.
- §9.3 partly: with the fork, a scan whose health says "no matched points" is not forwarded. Cross-checks
  against the Pixhawk IMU and the rangefinder are still not built.
