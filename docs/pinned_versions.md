<!-- filepath: docs/pinned_versions.md -->
# Pinned Third-Party Versions

| Component | Source | Pin | Rationale |
|---|---|---|---|
| px4_msgs | github.com/PX4/px4_msgs | `a1045ec4` (release/1.15) | Must match flashed PX4 v1.15.x firmware |
| livox_ros_driver2 | github.com/Livox-SDK/livox_ros_driver2 | `13eb05e4` (master) | Built 2026-07-04; params verified against source |
| FAST_LIO | github.com/hku-mars/FAST_LIO | `a4743b09` (ROS2) + ikd-Tree `e2e3f4e9` | ROS2 branch; config keys verified against its mid360.yaml |
| Micro-XRCE-DDS-Agent | github.com/eProsima/Micro-XRCE-DDS-Agent | tag v2.4.3 | v2.x pairs with stock v1.15 firmware client |
| Livox-SDK2 | github.com/Livox-SDK/Livox-SDK2 | master @ 2026-07-04 (installed to /usr/local) | Required by livox_ros_driver2 |

| Aerostack2 core | apt: ros-humble-as2-* | 1.1.3-1jammy (deb) | Official Humble debs; metapackage skipped (gazebo-assets ignition version conflict on this Jetson; sim not needed on vehicle) |
| as2_platform_pixhawk | github.com/aerostack2/as2_platform_pixhawk | `482563ba` (main @ 2026-07-05) | NOT released as deb — source-built. Upstream pins px4_msgs release/1.14; built against our 1.15 with `deploy/patches/as2_platform_pixhawk_px4msgs_1.15.patch` (3-line SensorGps field rename, GPS path unused on this GPS-denied vehicle). Upstream launch wrapper bypassed (fmu_prefix quoting bug), node launched directly from gate5_as2.launch.py. Second patch, applied after the first: `deploy/patches/as2_platform_pixhawk_offboard_hold.patch` (7 files, +1305/−29; new `offboard_guard.hpp/.cpp` and 2 gtest files). No setpoints outside Offboard (PX4 v1.15's uXRCE client publishes /fmu/in into the controllers in every mode), a hold on every entry, AS2 commands only after a control mode set after the request, none after an exit or disarm until restart. gate5_as2.launch.py refuses to start without it |

Additions for Phases 3–7 (EGO-Planner ports, FUEL/TARE candidates) are
appended here by their work packages with provenance and diff surface, per
masterplan §6/§7.
