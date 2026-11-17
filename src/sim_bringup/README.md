<!-- filepath: src/sim_bringup/README.md -->
# sim_bringup: closed-loop rig

Runs the real autonomy stack against a kinematic fake PX4 and a raycast LiDAR world, on the Jetson,
with no vehicle. Built 2026-11-16 (operator decision D1).

**It is not a flight simulator.** There is no attitude, thrust, vibration, EKF2 or FAST-LIO in it. It
exercises state machines, frames, interfaces and failure responses. It discharges no verification debt:
every closed-loop entry in `docs/verification_debt.yaml` stays open until the vehicle has done it.

| Real | Fake |
|---|---|
| `px4_odom_bridge`, `geofence_watchdog`, Aerostack2 (patched platform, estimator, controller, behaviours), `map_interface`, `lio_health_guard`, `go_to_with_avoidance`, `frontier_explorer` | PX4 (`fake_px4`), Mid-360 + FAST-LIO (`fake_lidar`) |

## Run

The launch file refuses to start unless `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID` is not 0, so it cannot
reach the flight controller. `PYTHONNOUSERSITE=1` is needed on this Jetson (numpy in `~/.local` breaks scipy).

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=73 PYTHONNOUSERSITE=1
export CYCLONEDDS_URI=file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds_bench.xml
ros2 launch sim_bringup sim.launch.py                  # terminal 1
ros2 run sim_bringup sim_mission                       # terminal 2: full exploration mission
ros2 run sim_bringup sim_mission --no-explore          # takeoff and land only

ros2 launch sim_bringup sim.launch.py explorer:=false  # for the scenarios, restart per scenario
ros2 run sim_bringup sim_scenarios hover_cancel        # | takeover | lio_jump | lio_silent
```

Stop the launch with Ctrl-C and check nothing is left: `pgrep -af "as2_|fake_px4|map_server"`.

## Results, 2026-11-16 (world `two_rooms`, Jetson Orin Nano)

| Run | Result |
|---|---|
| Takeoff and land through Aerostack2 | pass; 0 setpoints outside Offboard, 1 in transit at the exit |
| Exploration mission | DONE/COMPLETE, both rooms through a 1.0 m door, 22.2 m² free at 1.3 m, 0 unexplored, landed |
| `hover_cancel` | behaviour cancelled, vehicle stopped, 4 cm drift in 5 s, still in Offboard |
| `takeover` | pilot mode holds, no setpoint reaches PX4, a new goal is refused |
| `lio_jump` (3 m step in LIO) | bridge BLOCKED, watchdog lands, PX4 confirms, watchdog stops commanding |
| `lio_silent` | watchdog lands |

Found by the rig on its first run: the geofence watchdog and `lio_health_guard` both died on a change of
log severity at one call site (the watchdog on a soft breach followed by a hard one). Both fixed.

What the rig showed about Aerostack2 1.1.3 (still to be seen on the vehicle): the action is
`/drone0/FollowPathBehavior`, it accepts `frame_id: earth`, and cancelling it leaves the vehicle holding.

## Fork naming

`ros2 launch sim_bringup sim.launch.py lio:=fork` makes the fake LiDAR publish under the fork's topic
names and its health message (the fork's `interface` package must be sourced). 2026-11-16: mission
complete; `lio_jump` lands; `hover_cancel` holds; `lio_degenerate` (odometry alive, native health bad)
stops the planner and the vehicle holds without landing. The fork's filter is not in this loop.

## Test hooks

`/sim/pilot_takeover` (Empty), `/sim/kill` (Empty), `/sim/battery` (Float32, 0..1),
`/sim/lio_fault` (String: `silent`, `jump`, `ok`, and `degenerate` with `lio:=fork`), `/sim/px4_state` (String, read-only).
Worlds: `two_rooms`, `one_room_pillar` (`world:=`); the venue file is for `two_rooms`.
