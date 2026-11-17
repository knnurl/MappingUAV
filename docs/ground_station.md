<!-- filepath: docs/ground_station.md -->
# Watching the vehicle from the ground

RViz never runs on the vehicle. Three ways to see what it is doing, most robust first.

## 1. Status board over ssh (use this in flight)

```bash
ssh neric@<jetson-ip> 'python3 ~/colcon_ws/scripts/status_board.py'
```

It runs on the Jetson and only the terminal text crosses the WiFi, so the lab's access-point roaming, which
breaks DDS discovery mid-session, cannot break it. One screen: PX4 mode, Aerostack2 state, battery, odometry
bridge, LIO health, geofence, both position estimates, planner and explorer state, and the last alerts. A
line reads SILENT when its topic has not been heard for 3 s. Add `--lio-odom-topic /fastlio2/lio_odom` with
the fork. It is a display, not a safety function: the pilot's abort is the RC.

## 2. The saved map, during the flight

```bash
ssh neric@<jetson-ip> '~/colcon_ws/scripts/map_sync.sh <user>@<ground-ip>:/data/maps/'
```

Chunks of the 3D map arrive every 20 s; open them together in CloudCompare or `pcl_viewer`. rsync resumes
after a dropped link.

## 3. Live ROS graph on the ground station (map and goals in RViz)

Only if the network holds still. The roaming problem is DDS multicast discovery re-running when the
interface changes access point. Use a fixed access point or a cable, and unicast discovery:

- ground station and Jetson on the same subnet, `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`;
- on BOTH sides a CycloneDDS config whose `<Discovery><Peers>` lists the other machine's address and
  `<AllowMulticast>false</AllowMulticast>` [VERIFY on the lab network: not tested here];
- then `rviz2 -d ~/colcon_ws/tools/ground_view/exploration.rviz` shows the occupied cells, the saved map
  topic and the current goal.

Do not record or view `/cloud_registered` over WiFi in flight: it is the one large stream.
