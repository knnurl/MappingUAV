<!-- filepath: docs/lio_roadmap.md -->
# LIO Roadmap: odometry and mapping for the X500

Indoors there is no GPS, so LiDAR-inertial odometry is PX4's position source: FAST-LIO2's
`/Odometry` goes through `px4_odom_bridge` into EKF2. This document plans the LIO side for that
role. The full plan, with its evidence, is in the
[FASTLIO2_ROS2 roadmap](https://github.com/knnurl/FASTLIO2_ROS2/blob/main/ROADMAP.md); this
page covers what touches MappingUAV.

Priorities, in order:

1. **Never lose track.** A wrong or jumping pose goes straight into EKF2.
2. **Map accurately.** The exploration mission's product is the 3D map.

Status as of 2026-10-23. Gate status is unchanged; `docs/gate_status.yaml` remains the
operator's.

## Two LIO stacks

| | Upstream [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) (pinned here, `a4743b09`) | Fork [knnurl/FASTLIO2_ROS2](https://github.com/knnurl/FASTLIO2_ROS2) |
|---|---|---|
| Odometry | `/Odometry`, consumed by `px4_odom_bridge` today | `/fastlio2/lio_odom` |
| Health signal | none (`lio_health_guard` derives health from `/Odometry`) | localizer status; a LIO health signal is planned (B1) |
| Relocalization in a saved map | no | yes (`localizer`, global NDT+ICP search) |
| Loop closure / map refinement | no | `pgo` / `hba` |
| Tests and CI | none | unit tests, replay tool, GitHub Actions |

The fork's `tools/eval/eval.py` replays a recorded bag through either stack, using this repo's
`config/fast_lio_mid360_x500.yaml` for upstream. It reports the Gate 2 numbers (return-to-start
gap, height drift) plus CPU and memory.

## Decisions this repo will need

- **D1. Which LIO feeds `px4_odom_bridge`.** Keep upstream FAST-LIO and use the fork only for
  relocalization and map refinement, or switch the bridge to the fork's `lio_node` (topic
  `/fastlio2/lio_odom`). Decided from the recordings below, with both stacks at matched settings.
- **D2. Health input to `lio_health_guard`.** If the fork publishes a real degeneracy signal
  (planned), the guard can request a hover before `/Odometry` visibly breaks, instead of only
  reacting to rate drops and jumps.
- **D3. Nothing into EKF2 may jump.** Relocalization and loop-closure corrections move only a map
  frame, never the odometry the bridge sends. This should become a written rule beside the
  bridge's existing design notes, with a test.

## Recordings needed (Gate 2 style)

Hand-carried with the Mid-360 on the X500, props off. These are bench recordings: raw
`/livox/lidar` is still never recorded in flight. Record `/livox/lidar` and `/livox/imu`.

| Recording | Measures | Gate 2 criterion |
|---|---|---|
| 5-minute closed loop back to a marked pad | drift (`end_start_gap_m`) | return-to-start < 5 cm |
| 5 minutes held still | height drift (`z_max_dev_m`) | no height drift over 5 min |
| Height changes, stairs if available | height tracking | |
| Long corridor or open hall | degenerate geometry | |
| Fast yaw turns | rotation handling | |

Gate 3 still needs a live EKF2 run; recordings cannot stand in for it. The corridor recording
shows where LIO degrades, which is where Gate 3 should check that EKF2 holds position with the
HereFlow optical flow and the TF02 rangefinder.

## Parked

The fork's people filter, semantic labelling and Spot/VLP-16 configs are not part of this plan.
