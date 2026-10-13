<!-- filepath: docs/agent_handoff_2026-09-13.md -->
# Agent Handoff: Project State as of 2026-09-13

For the next agent session. Read this first, then `README.md`, `docs/gate_status.yaml` (local), and the governing specs in `docs/ProgressReport/`.

## 1. Operator rules (do not break)
1. **`docs/gate_status.yaml` is operator-only.** Never edit it. It currently has local, **uncommitted** changes (`decision_dp1_mapping: CPU_GRID`, `decision_dp2_explore: FRONTIER`) that **stay local**; don't commit them.
2. **Commit and push only when the operator asks.** Stage explicit paths; never `git add -A`.
3. **`docs/ProgressReport/` is private** (chat backups, masterplan, handover specs, architectural review). Never commit or publish it. The GitHub repo `knnurl/MappingUAV` is **public**.
4. **The masterplan must not be edited.** The operator granted a one-time edit (amendment A1, adding FRONTIER to DP-2) on 2026-09-13; it is used up.
5. **The agent never touches the vehicle.** Anything physical is a human-executed runbook. The agent may run read-only checks and the agent-side verification scripts (e.g. `verify_gate1.sh`).
6. **No self-certification:** report gate evidence; the operator sets tokens.
7. Global rules: file path header in every new file; `colcon build --parallel-workers 2` with `MAKEFLAGS=-j4`; no RViz on the vehicle.

## 2. Where the project is
| Gate | State |
|---|---|
| 1 uXRCE-DDS link | **Evidence complete, token pending (operator).** Transcript: `docs/gate1_verification_2026-09-13.md` (uncommitted) |
| 2 LiDAR bench | Software ready except network; Mid-360 **not mounted yet** |
| 3–7 | PENDING. Phase 3–7 software built on `wp-*` branches; not merged (by design). |

Hardware: Pixhawk 6C reflashed to PX4 v1.15.0 (record: `docs/px4_reflash_runbook.md`). TELEM2 ↔ Jetson via **FTDI TTL-232R-3V3 USB cable** (`/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTCKEJ6W-if00-port0`); the operator plans a better cable. QGC runs on the Jetson in a container (`scripts/run_qgc.sh`), talking to the FC over its own USB (`/dev/ttyACM0`, don't open it while QGC runs).

## 3. Decisions in force
| Topic | Decision |
|---|---|
| DP-1 mapping | CPU_GRID (OctoMap) |
| DP-2 exploration | FRONTIER (in-house explorer). Spec `docs/wp-f_frontier_explorer_spec.md`; verified prototype `tools/frontier_prototype/` |
| OI-1 | **Declined:** no `frontier_explorer` code until Gate 5 passes and `map_interface` is merged to `main`; then branch `wp-f-frontier` from `main` |
| ROS domain | **0** on the Jetson (matches PX4 `UXRCE_DDS_DOM_ID` default; FC unchanged) |
| Ground link | WiFi (`wlP1p1s0`, network "Radiate"). Ethernet port is reserved for the Mid-360 |
| D435i | Color + depth views only, pointcloud off, depth decimation ×2; not a mapping input |
| `EKF2_EV_DELAY` | 5.0 ms on the FC (residual only; tune at Gate 3) |
| Bridge | `px4_odom_bridge` stamps `timestamp_sample` with the LIO scan time; drops stale/future stamps; logs other EV publishers. Frame: ENU +x (startup forward) = PX4 East |

## 4. Environment gotchas (learned the hard way)
- **CycloneDDS:** `cyclonedds.xml` lists loopback plus WiFi with `presence_required="false"`. A required-but-absent interface makes every node fail with `rmw handle is invalid`.
- **`ros2 topic hz` can't read PX4 `/fmu/out/*`:** Humble's `hz` has no QoS options, PX4 publishes BEST_EFFORT. Use `scripts/fmu_rate_probe.py`. `ros2 topic echo --qos-reliability best_effort` works.
- **Cold ROS daemon** returns partial topic lists; use `ros2 topic list --no-daemon --spin-time 5`.
- **`set -e` + `timeout`:** `timeout` exits 124 and aborts the script. Pipe to `tail` or add `|| true`.
- **FTDI latency timer** defaults to 16 ms and biased timesync by > 10 ms. Set to 1 ms now; persistent udev rule **not yet installed** (correct path `/etc/udev/rules.d/`, see the Gate 1 transcript).
- **Timesync over USB-serial:** residual ±4 ms (half the round trip); FC clock drifts ~33–60 ppm while warming up. Warm up ~5 min before timing-sensitive recordings.
- **Livox driver ignores Ctrl-C** after a failed start; launch escalates to SIGKILL after ~15 s.
- **`px4_odom_bridge` install is stale** (built 2026-07-04, before commit 20e851e). Rebuild before Gate 3.
- `drone_bringup` configs and launch files in `install/` are symlinks to `src/`; edits apply without rebuild (new files need a rebuild).
- **Jetson kernel** has only `ftdi_sio` and `cp210x` USB-serial drivers (no CH340/PL2303).
- **`pkill -f <pattern>` matches the calling shell's own command line** and kills it; use `pgrep -x <name>`.
- **graphify:** `octomap/` is excluded in `.graphifyignore` (it was 78% of the graph). Run `graphify update .` after code changes.
- Sandbox: node tests need no special DDS flags now that WiFi is optional in `cyclonedds.xml`.

## 5. Repository state
Remote: `https://github.com/knnurl/MappingUAV` (public). Local checkout: branch `main`.

Recent commits on `main` (newest first): `357f95e` Gate 1 rate probe, fixed verify scripts, domain 0 · `f6d69df` DDS WiFi optional · `29bd04f` spec OI-1 declined · `cacfcc5` reflash record · `420a7b2` README DP-2 · `8c0c428` explorer spec + prototype · `c7bf736`/`c2bf60d` D435i · `20e851e` bridge timestamps + sole-writer logging + Gate 3 sign test.

Branch heads: `wp-b-as2` `cada09b` (gate5 launch refuses `external_odom != false`), `wp-c-mapping` `a373313`, `wp-d-planner` `11f7339`, `wp-e-geofence-watchdog` `ed4443f` (geofence axis note), `wp-f-dp2-survey` `a4ec668`, `wp-g-ops` `f7308bc` (domain 0). `qgc-jetson-container` is merged. The `wp-*` branches have drifted from `main`; expect merge conflicts in `drone_bringup` and docs.

**Uncommitted in the working tree** (operator has not asked to commit):
- `docs/gate_status.yaml` (operator's, stays local)
- `docs/gate1_verification_2026-09-13.md` (operator said: don't commit)
- `docs/agent_handoff_2026-09-13.md` (this file)
- Gate 2 fixes: `scripts/verify_gate2.sh`, `scripts/verify_gate3.sh`, `scripts/record_bench_bag.sh`, `test_data/manifest.yaml`
- `git stash@{0}`: operator's pre-edit `d435i.launch.py` (superseded by `main`; can be dropped)

Local system changes (not in git): `~/.bashrc` `ROS_DOMAIN_ID=0`; WiFi radio unblocked; FTDI `latency_timer` = 1 until replug.

## 6. Next steps
**Gate 1 (operator):** set the token; install the persistent FTDI udev rule; after the cable swap, re-run `verify_gate1.sh` and re-measure timesync.

**Gate 2:**
1. **Network (not done; needs operator OK, it's a system change):** dedicate Ethernet `enP8p1s0` to the LiDAR with static `192.168.1.5/24`, no gateway, e.g.
   `nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv4.gateway "" ipv4.never-default yes ipv6.method disabled && nmcli connection up "Wired connection 1"`.
   WiFi keeps internet. The operator is not SSH'd over Ethernet.
2. Operator mounts and powers the Mid-360 (IP `192.168.1.123`), foam standoffs, nothing within 0.5 m blind radius; measures LiDAR→FC lever arm while there (needed at Gate 3).
3. `ros2 launch drone_bringup gate2_lio.launch.py`, then `scripts/verify_gate2.sh`, then the manual walk test with `scripts/record_bench_bag.sh gate2 walk` and a `test_data/manifest.yaml` entry.

**Before Gate 3:** rebuild `px4_odom_bridge`; resolve which `distance_sensor` instance EKF2 uses (VL53 would cap height aiding at 0.65 m; blocking); measure and apply the LiDAR lever arm; `reset_counter` handling in the bridge (not done); tune `EKF2_EV_DELAY` from logs; consider the 40-pin UART if timesync noise shows in EV fusion.

**Software backlog (no token needed):** fix OI-11 on `wp-d-planner` (`config/planning/` not installed); consolidate `verification_debt.yaml` (VD-001…006 spread over branches) onto `main` before Gate 5; write Gate 5/6 runbooks with open debt on page one; keep `wp-*` branches close to `main`.

**Before Gate 7:** size planning box, map bounds and geofence together for the venue (explorer spec OI-2); venue survey for doorway widths and glass.

## 7. Key documents
| Doc | What |
|---|---|
| `README.md` | Public project overview |
| `docs/gate_status.yaml` | Gate tokens and decisions (operator-only, local) |
| `docs/gate1_verification_2026-09-13.md` | Gate 1 evidence |
| `docs/px4_reflash_runbook.md` | Reflash record and carried-forward items |
| `docs/wp-f_frontier_explorer_spec.md` | Explorer spec v3.1 (verified twice) |
| `tools/frontier_prototype/` | Reference prototype and spec check scripts |
| `docs/pinned_versions.md`, `deploy/third_party.repos` | Third-party pins (as2 platform needs `deploy/patches/…px4msgs_1.15.patch`) |
| `docs/ProgressReport/` (private) | Masterplan (with amendment A1), Gates 1–4 handover spec, architectural review (§0 retracted) |
