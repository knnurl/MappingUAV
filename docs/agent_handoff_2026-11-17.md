<!-- filepath: docs/agent_handoff_2026-11-17.md -->
# Agent handoff: project state as of 2026-11-17

Read this first in a new session. The operator rules of `agent_handoff_2026-09-13.md` §1 still bind
(gate tokens are the operator's; commit only when asked; never touch the vehicle).

## State

| | |
|---|---|
| Gates | 1, 2, 3 passed on the bench. 4 (tethered hover) next. Nothing has flown. |
| Critical path | Hardware: `docs/gate4_runbook.txt`, starting with its Part 0 (quiet bench run, LiDAR power cut, fresh parameter dump). |
| Software | Built through exploration, unverified on the vehicle. Integration branch: **`wp-h-sim`** (later gates merge from it). |
| LIO | Upstream FAST-LIO flies Gate 4. The fork swaps in afterwards with `lio:=fork` and `docs/lio_swap_runbook.txt` (on `wp-h-sim`). |

## Where things are (branch `wp-h-sim` unless noted)

| What | Where |
|---|---|
| Single flight bringup, layers per gate | `src/drone_bringup/launch/flight.launch.py` |
| Pre-flight check (read-only) | `scripts/preflight_check.py` |
| Ground view over ssh | `scripts/status_board.py`, `docs/ground_station.md` |
| Venue sizing: every box from one file | `src/drone_bringup/config/venue.yaml` + `scripts/venue_boxes.py` (values are placeholders) |
| Closed-loop rig (real stack, fake PX4 and LiDAR) | `src/sim_bringup/` (README has commands and results) |
| Runbooks | `docs/gate4_runbook.txt` (main), `gate5_as2_runbook.txt`, `gate6_planner_runbook.txt`, `gate7_watchdog_trip_runbook.txt`, `gate7_exploration_runbook.txt`, `lio_swap_runbook.txt` |
| Who decides what when LIO fails | `docs/safety_ownership.md` |
| Later upgrades (map refinement, altitude band, EGO-Planner) | `docs/later_upgrades_spec.md` |
| Open verification debt | `docs/verification_debt.yaml` (12 entries on `wp-h-sim`) |
| Tests | `scripts/run_pure_tests.sh` (no ROS; also CI), `scripts/run_all_tests.sh` (Jetson) |
| FC parameters expected before Gate 4 | `scripts/param_check.py`, `src/drone_bringup/config/px4/gate4_expected.params` (main) |
| LIO fork health signal | repo knnurl/FASTLIO2_ROS2, branch `b1-health`, `docs/lio_health.md` |

## Things that bite

- **Scipy on the Jetson**: `~/.local` has numpy 2, which breaks the system scipy. Run anything using the
  planner or explorer with `PYTHONNOUSERSITE=1` (the launch files and scripts set it).
- **rclpy logging**: one call site must not log at two severities (it raises). This killed the watchdog on a
  soft-then-hard breach until 2026-11-16. A contract test now scans for the pattern.
- **Never name an rclpy `Node` method `handle`**: it shadows the node's internal handle.
- **After any Offboard exit** the Aerostack2 platform forwards nothing until restarted
  (`scripts/restart_as2.sh`, or restart terminal T3).
- **Node tests** need the isolated environment in `scripts/run_all_tests.sh` and the XRCE agent stopped.
- **Killing ROS processes**: `kill` of a `ros2 run`/`ros2 launch` wrapper leaves the node alive; kill the node
  binary. A `pkill -f` pattern that also appears in your own command line kills your shell.
- **The rig is not a simulator**: no flight dynamics, no EKF2, no FAST-LIO. It discharges no debt.

## Waiting on the operator

1. Gate 4 hardware items and Part 0 of its runbook.
2. Real venue dimensions.
3. Review and merge of the fork's `b1-health`; after Gate 4, the swap runbook.
4. Recordings where LIO degrades (plain corridor, stairs, props on): the health thresholds are provisional.
5. HereFlow mounting measurement, to get a second velocity source back.
6. Decisions L1 to L4 in `docs/later_upgrades_spec.md`.
7. Three files are deleted in the working tree of `main` but not committed
   (`docs/px4_reflash_runbook.md`, `docs/gate1_px4_console.txt`,
   `docs/ardupilot_4.5.4_params_pre_reflash.txt`); the README still links the first. Restore or commit the deletion.
