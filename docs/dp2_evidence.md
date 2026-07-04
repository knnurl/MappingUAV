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
  3. **Compile execution is PENDING OPERATOR APPROVAL:** the agent's
     environment permission layer blocked building the third-party tree with
     swapped-in binary libraries (untrusted-code-integration class). Working
     tree with the swap prepared under the session scratchpad
     (`tare_planner/` + `or-tools_aarch64_...`). No verdict is claimed.

## Current standing (agent, non-binding)
TARE is ahead on every measurable axis: official ROS2 branches, active
maintenance, aerial companion environment. FUEL's lack of any ROS2 port
makes it a porting project, not an adoption. A final recommendation awaits
the TARE compile-health verdict on aarch64 (operator to approve the build)
and, ideally, the deferred coverage benchmark.

Decision belongs to the human: set `decision_dp2_explore` in
docs/gate_status.yaml.
