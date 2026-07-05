<!-- filepath: docs/dp2_evidence.md -->
# DP-2 Evidence: Exploration Planner (FUEL vs TARE)

**Evidence quality caveat (required first paragraph):** the full coverage
benchmark is DEFERRED — it needs the sim rig that is explicitly not built in
this phase, and is logged as verification debt (VD-003). This document
covers port state, maintenance, and build-health groundwork only. Status:
EVIDENCE_ONLY.

## FUEL (HKUST-Aerial-Robotics/FUEL)
- Upstream is ROS1 (catkin) only. No official ROS2 branch.
- No credible community ROS2 Humble port found (2026-07-05 search; the
  hku-mars ecosystem's ROS2 momentum is around SUPER/ROG-Map, which are
  different planners, not FUEL).
- Compile-health on Humble/aarch64: **N/A — nothing to build.** Adopting
  FUEL would mean a full ROS1->ROS2 port of an estimation-coupled planner,
  exactly what masterplan §6 deprioritizes ("port from ROS1 upstream only if
  all candidates fail").

## TARE (caochao39/tare_planner, CMU exploration ecosystem)
- **Official `humble-jazzy` branch exists** on tare_planner, and the aerial
  companion repo (`aerial_navigation_development_environment`) has an
  official `humble` branch. Actively maintained; DARPA SubT pedigree.
- Aerial support: A-TARE / the aerial navigation environment target drones;
  base tare_planner targets ground vehicles — the aerial variant path needs
  hands-on evaluation.
- Build-health findings so far (session 1):
  1. The repo BUNDLES or-tools 9.8 binaries for **x86-64 only** — it cannot
     link on the Orin Nano as shipped.
  2. Google publishes an arm64 or-tools v9.8 tarball (debian-11 asset; glibc
     2.31 binaries, expected compatible with Ubuntu 22.04). Swapped into the
     source tree cleanly (include/ + lib/).
  3. **Compile-health verdict (operator-approved, executed 2026-07-05):
     PASS.** `humble-jazzy` branch builds clean on Humble/aarch64 in 5m18s
     (warnings only, zero errors) with Google's arm64 v9.8 or-tools tarball
     (debian-11 asset) swapped into `src/tare_planner/or-tools/`; the
     resulting `tare_planner_node` fully resolves all shared libraries on
     Ubuntu 22.04 (glibc 2.35 covers 2.31). The or-tools swap is the only
     modification — record it as a required import step if TARE is chosen.

## Current standing (agent, non-binding)
**TARE.** Official ROS2 branches, active maintenance, aerial companion
environment, and a PASSING compile-health verdict on this exact board. FUEL
has no ROS2 port at all — adopting it means porting it. Remaining unknowns
for TARE are behavioral (aerial variant fit, coverage efficiency), which sit
behind the deferred sim benchmark (VD-003).

Decision belongs to the human: set `decision_dp2_explore` in
docs/gate_status.yaml.
