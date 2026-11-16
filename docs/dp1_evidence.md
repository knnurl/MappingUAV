<!-- filepath: docs/dp1_evidence.md -->
# DP-1 Evidence: Mapping Backend (NVBLOX vs CPU_GRID)

**Evidence quality caveat (required first paragraph):** no Gate 2/3 bags
exist yet. Everything below is (a) source/documentation research and (b)
benchmarks against PROCEDURALLY GENERATED room+pillar clouds
(`tools/replay_tests/synthetic_scene.py`), not real Mid-360 data. Treat the
CPU numbers as optimistic lower bounds and the nvblox findings as
research-grade. Re-run `tools/replay_tests/` against real bags before
deciding, if at all possible. Logged as VD-002.

## Session log (bounded: 2 working sessions max, masterplan §5)
- **Session 1 (2026-07-04/05):** research + CPU backend implementation +
  synthetic benchmark. This document.
- Session 2: not yet spent. Reserve for an nvblox hands-on attempt ONLY if
  the human wants it despite the findings below.

## CPU_GRID backend (implemented, tested)
- octomap v1.10.0 + dynamicEDT3D, vendored source-pinned (Bonxai's ROS2
  wrapper verified immature 2026-07-04: upstream calls it educational/under
  development, occupancy pipeline incomplete, not on the build farm — the
  masterplan's own fallback condition triggered, OctoMap selected).
- Bounded box, 0.2 m voxels, 2 Hz insertion with 8 m range cut, ESDF
  clearance capped at 4 m.
- Synthetic replay results (Orin Nano, max clocks): 5/5 harness tests pass;
  81-point occupancy+clearance service query answered well inside the 200 ms
  budget; process survives full replay. CPU/RSS: see docs/compute_ledger.md.

## NVBLOX backend (research only, not attempted hands-on)
Findings against masterplan §5's stated risk:
1. **LiDAR model mismatch (the blocker):** isaac_ros_nvblox's LiDAR path
   assumes a structured spinning-lidar beam pattern (fixed height x width
   equirectangular projection; Ouster OS1 examples). The Mid-360's
   non-repetitive rosette pattern does not produce organized scans; there is
   no documented Livox ingestion path. A custom reprojection layer
   (accumulate + rebin into a virtual spinning lidar) would be new, untested,
   estimation-critical code.
2. **Platform:** Humble+Jetson is supported via the Isaac ROS distribution
   (JetPack 6 aarch64), but the recommended install path is the Isaac ROS
   CLI/container stack — a heavyweight dependency on an 8 GB board that
   already carries FAST-LIO.
3. **Budget fit:** nvblox shines with dense depth cameras and GPU ESDF at
   high rates; our consumer is a 0.2 m coarse advisory grid at 2 Hz. The
   performance headroom it buys is not needed by any current requirement.

## Recommendation (agent, non-binding)
**CPU_GRID.** The masterplan's bounded-pursuit rule ("if a clean port is not
achieved within 2 sessions, recommend CPU_GRID") is effectively pre-decided
by finding 1: a clean port is not plausible inside the budget. Session 2 is
better spent elsewhere unless the human explicitly wants the nvblox attempt.

Decision belongs to the human: set `decision_dp1_mapping` in
docs/gate_status.yaml. The losing branch is archived, not deleted.
