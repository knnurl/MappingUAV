<!-- filepath: docs/build_report_gates1-4.md -->
# Build & Discrepancy Report — Gates 1–4 Infrastructure
Date: 2026-07-04 · Machine: Jetson Orin Nano 8GB, L4T R36.5 (JetPack 6), Ubuntu 22.04.5

## Result
Workspace `~/colcon_ws` fully provisioned and built. All 5 packages compile:
px4_msgs (release/1.15, 11m06s) · livox_ros_driver2 (42s) · fast_lio (2m54s) ·
px4_odom_bridge · drone_bringup. Micro XRCE-DDS Agent v2.4.3 and Livox SDK2
installed to /usr/local. ROS 2 Humble desktop installed (was absent).

## [VERIFY] outcomes (spec section references)
| Item | Result |
|---|---|
| px4_msgs branch vs firmware | release/1.15 per operator-confirmed v1.15.x firmware |
| FAST-LIO odometry topic | `/Odometry` confirmed in laserMapping.cpp:934 |
| FAST-LIO launch argument (§4.2) | **Spec draft wrong**: takes `config_path` (dir) + `config_file` (filename), not a full path. gate2_lio.launch.py corrected. |
| fast_lio_mid360_x500.yaml keys | Matched repo mid360.yaml structure; `timestamp_unit: 3` confirmed. Repo has 3 keys spec omitted: `map_file_path`, `scan_rate: 10`, `effect_map_en`/`map_en` — kept, memory-safe values (`map_en: false`). |
| livox_ros_driver2 params/executable | All names match source; executable `livox_ros_driver2_node` confirmed. |
| Mid-360 extrinsic [FIXED] values | Identical in repo config. |
| XRCE agent version | v2.4.3 (stock v1.15 firmware → v2.x correct). |

## Deviations (reported per global rule 5 / §6)
1. **ROS 2 not preinstalled** (spec assumed it). Installed `ros-humble-desktop`
   + `ros-dev-tools` per operator decision. setup_env.sh §0b updated.
2. **livox build.sh not executed.** It wipes `build/`+`install/` of the whole
   workspace and runs colcon at default parallelism (violates rule 2, OOM risk).
   Replicated its exact side effects instead: `cp package_ROS2.xml package.xml`,
   staged `launch_ROS2/`→`launch/` for the build, built with its defines
   `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`, removed staged `launch/` after.
   Additionally `~/.colcon/defaults.yaml` now caps ALL colcon runs at
   `parallel-workers: 2` + `symlink-install` as a backstop.
3. **File-header rule vs file formats:** XML files carry the filepath comment
   after the `<?xml?>` declaration (comment-first is invalid XML). JSON has no
   comments: MID360_config.json uses a leading `"_filepath"` key (ignored by
   the driver's parser).
4. **nvgetty does not exist on JetPack 6** — no getty holds /dev/ttyTHS1;
   spec intent already satisfied. Power mode 0 = 15W on this board (applied,
   with jetson_clocks).
5. **cyclonedds.xml interface**: `wlP1p1s0` (verified via `ip link`), not `wlan0`.

## Environment facts
- Mid-360 unit IP: **192.168.1.123** (operator-read serial sticker).
- CycloneDDS bound to `lo` + `wlP1p1s0`; `ROS_DOMAIN_ID=42`; env in ~/.bashrc.
- UDP buffers set via /etc/sysctl.d/60-cyclonedds.conf (8 MB).
- `neric` added to `dialout` — **takes effect on next login**.

## Smoke tests (no flight hardware attached)
- `odom_bridge_node`: starts, logs `Bridging /Odometry -> /fmu/in/vehicle_visual_odometry`,
  5-s no-data health warning fires. PASS.
- `gate2_lio.launch.py`: Livox driver parses MID360_config.json
  ("successfully parse base config, counts: 1"); expected `bind failed` since
  host isn't on 192.168.1.5 yet. FAST-LIO reads yaml (`lidar_type 1`), node
  init finished. PASS (startup-level).
- `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600`: transport init OK. PASS.
- gate3 static TF publishers: argument style accepted, 9.99 placeholders
  publish. PASS.

## Outstanding before Gate 1/2 can PASS (physical/operator steps)
1. Wire Pixhawk TELEM2 ↔ Jetson 40-pin UART (ttyTHS1: pin8 TX, pin10 RX, cross, GND).
2. Apply `config/px4/gate3_ekf2.params` via QGC (UXRCE_DDS_CFG=TELEM2, etc.), reboot FC.
3. Connect Mid-360, then switch Ethernet to the lidar subnet:
   `sudo nmcli con add type ethernet ifname enP8p1s0 con-name livox-static ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv4.gateway "" && sudo nmcli con up livox-static`
4. Log out/in (dialout group), then run `scripts/verify_gate1.sh` / `verify_gate2.sh`.
5. [MEASURE] items still open: lidar mount offset for gate3 TF (9.99 placeholders),
   `SENS_FLOW_ROT` for HereFlow.
